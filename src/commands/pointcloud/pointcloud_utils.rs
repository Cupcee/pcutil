use anyhow::{bail, Context, Result};
use bytemuck;
use pasture_core::layout::attributes;
use pasture_core::layout::PointLayout;
use pasture_core::nalgebra::{Matrix3, SymmetricEigen, Vector3};
use pasture_core::{
    containers::BorrowedBuffer, containers::BorrowedBufferExt, containers::BorrowedMutBuffer,
    containers::OwningBuffer, containers::VectorBuffer,
};
use pcd_rs::{DataKind, DynRecord, Field, Schema, ValueKind, WriterInit};
use ply_rs::ply::{Addable, Property, PropertyType, ScalarType};
use walkdir::WalkDir;

use crate::shared::math::{hull_volume_area, sample_for_hull};
use crate::DynFieldType;
use pasture_io::base::read_all;
use pcd_rs::DynReader;
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufReader, BufWriter};
use std::path::Path;

/// Struct to hold the summary of a point cloud file.
#[derive(Debug)]
pub struct PointcloudSummary {
    pub file_name: String,
    pub total_points: usize,
    // Bounding box
    pub min_x: f64,
    pub max_x: f64,
    pub min_y: f64,
    pub max_y: f64,
    pub min_z: f64,
    pub max_z: f64,
    // Statistical measures for positions
    pub mean_x: f64,
    pub mean_y: f64,
    pub mean_z: f64,
    // Density: points per cubic meter
    pub density: f64,
    // Classification counts if available
    pub classification_counts: Option<HashMap<u8, usize>>,
    // Sums for weighted averages (for aggregation purposes)
    pub sum_x: f64,
    pub sum_y: f64,
    pub sum_z: f64,
    pub bbox_volume: f64,
    pub convex_hull_volume: f64,
    pub convex_hull_area: f64,
    pub bbox_utilisation: f64,     // hull / bbox (0‒1)
    pub pca_eigenvalues: [f64; 3], // λ₁ ≥ λ₂ ≥ λ₃
}

impl PointcloudSummary {
    /// Compute summary statistics for a given point cloud buffer.
    /// This function collects per-file results which can later be aggregated in summary.rs.
    pub fn from(file_name: String, buffer: &VectorBuffer) -> Result<Self> {
        let total_points = buffer.len();
        if total_points == 0 {
            bail!("Buffer has no points to summarize.");
        }

        // Vectors to collect coordinate values and radii.
        let mut xs = Vec::with_capacity(total_points);
        let mut ys = Vec::with_capacity(total_points);
        let mut zs = Vec::with_capacity(total_points);
        let mut radii = Vec::with_capacity(total_points);

        // Initialize bounding box.
        let mut min_x = f64::MAX;
        let mut max_x = f64::MIN;
        let mut min_y = f64::MAX;
        let mut max_y = f64::MIN;
        let mut min_z = f64::MAX;
        let mut max_z = f64::MIN;

        // Process POSITION_3D.
        if buffer
            .point_layout()
            .has_attribute(&attributes::POSITION_3D)
        {
            let view = buffer.view_attribute::<Vector3<f64>>(&attributes::POSITION_3D);
            for p in view.into_iter() {
                xs.push(p.x);
                ys.push(p.y);
                zs.push(p.z);
                let r = (p.x * p.x + p.y * p.y + p.z * p.z).sqrt();
                radii.push(r);

                if p.x < min_x {
                    min_x = p.x;
                }
                if p.x > max_x {
                    max_x = p.x;
                }
                if p.y < min_y {
                    min_y = p.y;
                }
                if p.y > max_y {
                    max_y = p.y;
                }
                if p.z < min_z {
                    min_z = p.z;
                }
                if p.z > max_z {
                    max_z = p.z;
                }
            }
        } else {
            bail!("Buffer does not contain POSITION_3D attribute.");
        }

        // Compute sums for x, y, z and the means.
        let sum_x: f64 = xs.iter().sum();
        let sum_y: f64 = ys.iter().sum();
        let sum_z: f64 = zs.iter().sum();

        let mean_x = sum_x / total_points as f64;
        let mean_y = sum_y / total_points as f64;
        let mean_z = sum_z / total_points as f64;

        // Compute density: points per cubic meter.
        let dx = max_x - min_x;
        let dy = max_y - min_y;
        let dz = max_z - min_z;
        let bbox_volume = dx * dy * dz;

        // 1. Voxel edge length l  ≈  2 × average point spacing
        let avg_spacing = (bbox_volume / total_points as f64).cbrt();
        let l = 2.0 * avg_spacing;
        let l3 = l * l * l; // voxel volume

        // 2. Count points per voxel (sparse hashmap)
        let mut voxel_counts: HashMap<(i32, i32, i32), u32> = HashMap::new();
        for i in 0..total_points {
            let ix = ((xs[i] - min_x) / l).floor() as i32;
            let iy = ((ys[i] - min_y) / l).floor() as i32;
            let iz = ((zs[i] - min_z) / l).floor() as i32;
            *voxel_counts.entry((ix, iy, iz)).or_default() += 1;
        }

        // 3. Convert to density (points per m³)
        let occupied_voxels = voxel_counts.len() as f64;
        let density = (total_points as f64 / occupied_voxels) / l3;

        // PCA
        let mut cov = Matrix3::<f64>::zeros();
        for i in 0..total_points {
            let dx = xs[i] - mean_x;
            let dy = ys[i] - mean_y;
            let dz = zs[i] - mean_z;
            cov[(0, 0)] += dx * dx;
            cov[(0, 1)] += dx * dy;
            cov[(0, 2)] += dx * dz;
            cov[(1, 1)] += dy * dy;
            cov[(1, 2)] += dy * dz;
            cov[(2, 2)] += dz * dz;
        }
        // Upper triangle mirror
        cov[(1, 0)] = cov[(0, 1)];
        cov[(2, 0)] = cov[(0, 2)];
        cov[(2, 1)] = cov[(1, 2)];
        cov /= total_points as f64;

        let eig = SymmetricEigen::new(cov);
        let mut eigvals = eig.eigenvalues.as_slice().to_owned();
        eigvals.sort_by(|a, b| b.partial_cmp(a).unwrap()); // λ₁≥λ₂≥λ₃

        // Stats derived from Convex Hull
        let bbox_volume = dx * dy * dz;
        let sample = sample_for_hull(&xs, &ys, &zs);
        let (hull_volume, hull_area) = hull_volume_area(sample);
        let bbox_utilisation = if bbox_volume > 0.0 {
            (hull_volume / bbox_volume).clamp(0.0, 1.0)
        } else {
            0.0
        };

        // Classification counts if available.
        let classification_counts = if buffer
            .point_layout()
            .has_attribute(&attributes::CLASSIFICATION)
        {
            let mut counts = HashMap::new();
            let view = buffer.view_attribute::<u8>(&attributes::CLASSIFICATION);
            for val in view.into_iter() {
                *counts.entry(val).or_insert(0) += 1;
            }
            Some(counts)
        } else {
            None
        };

        Ok(Self {
            file_name,
            total_points,
            min_x,
            max_x,
            min_y,
            max_y,
            min_z,
            max_z,
            mean_x,
            mean_y,
            mean_z,
            density,
            classification_counts,
            sum_x,
            sum_y,
            sum_z,
            bbox_volume,
            convex_hull_volume: hull_volume,
            convex_hull_area: hull_area,
            bbox_utilisation,
            pca_eigenvalues: [eigvals[0], eigvals[1], eigvals[2]],
        })
    }
}

/// Determine file extension in lowercase.
pub fn extension(path: &str) -> String {
    Path::new(path)
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_lowercase()
}

/// Returns true if the provided extension is supported.
pub fn is_supported_extension(ext: &str) -> bool {
    matches!(ext, "las" | "laz" | "pcd" | "ply")
}

/// Read a single pointcloud file (.las/.laz, .pcd or .ply) into a VectorBuffer.
pub fn read_pointcloud_file_to_buffer(
    path: &str,
    factor: f64,
    dynamic_fields: Vec<DynFieldType>,
) -> Result<VectorBuffer> {
    let ext = Path::new(path)
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("");
    match ext.to_lowercase().as_str() {
        "las" | "laz" => {
            let buffer = read_all::<VectorBuffer, _>(path)?;
            Ok(buffer)
        }
        "pcd" => read_dyn_pcd_file(path, factor, dynamic_fields),
        "ply" => read_ply_file(path, factor),
        _ => bail!("Unsupported format: {}", path),
    }
}

/// Takes a dynamic field value, and attempts to read it as: u8 > u16 > i32.
/// Then attemps to fit the value into u8 and return the value.
/// Raises error, if the value does not fit into u8.
fn dyn_field_as_u8(field: &Field) -> Result<u8> {
    // Try direct u8 first, then fall back to float -> u8.
    if let Some(v) = field.to_value::<u8>() {
        Ok(v)
    } else if let Some(vu) = field.to_value::<u16>() {
        Ok(u8::try_from(vu).context("field u16 doesn't fit in u8")?)
    } else if let Some(vi) = field.to_value::<i32>() {
        Ok(u8::try_from(vi).context("field i32 doesn't fit in u8")?)
    } else if let Some(vf) = field.to_value::<f32>() {
        Ok(vf as u8)
    } else {
        bail!("Unsupported type for u8-compatible field")
    }
}

fn dyn_field_as_u16(field: &Field) -> Result<u16> {
    if let Some(v) = field.to_value::<u16>() {
        Ok(v)
    } else if let Some(vu) = field.to_value::<u32>() {
        Ok(u16::try_from(vu).context("field u32 doesn't fit in u16")?)
    } else if let Some(vi) = field.to_value::<i32>() {
        Ok(u16::try_from(vi).context("field i32 doesn't fit in u16")?)
    } else if let Some(vf) = field.to_value::<f32>() {
        Ok(vf as u16)
    } else {
        bail!("Unsupported type for u16-compatible field")
    }
}

fn dyn_field_as_f64(field: &Field) -> Result<f64> {
    if let Some(v) = field.to_value::<f64>() {
        Ok(v)
    } else if let Some(vf) = field.to_value::<f32>() {
        Ok(vf as f64)
    } else if let Some(vi) = field.to_value::<i32>() {
        Ok(vi as f64)
    } else {
        bail!("Unsupported type for f64-compatible field")
    }
}

/// Read a .pcd file without schema into a VectorBuffer with POSITION_3D.
/// Bails if unable to read the file.
pub fn read_dyn_pcd_file(
    path: &str,
    factor: f64,
    mut dynamic_fields: Vec<DynFieldType>,
) -> Result<VectorBuffer> {
    let reader = DynReader::open(path)?;
    
    // Auto-detect standard fields if dynamic_fields is empty
    if dynamic_fields.is_empty() {
        for field_def in reader.meta().field_defs.fields.iter().skip(3) {
            match field_def.name.to_lowercase().as_str() {
                "intensity" => dynamic_fields.push(DynFieldType::Intensity),
                "rgb" | "rgba" => dynamic_fields.push(DynFieldType::Color),
                "label" | "classification" => dynamic_fields.push(DynFieldType::Classification),
                "gps_time" | "timestamp" => dynamic_fields.push(DynFieldType::GpsTime),
                "user_data" => dynamic_fields.push(DynFieldType::UserData),
                "point_source_id" => dynamic_fields.push(DynFieldType::SourceID),
                "return_number" => dynamic_fields.push(DynFieldType::ReturnNumber),
                "number_of_returns" => dynamic_fields.push(DynFieldType::NumberOfReturns),
                _ => dynamic_fields.push(DynFieldType::Skip),
            }
        }
    }

    let points = reader.collect::<Result<Vec<_>, _>>()?;
    if points.is_empty() {
        bail!("No points found in the PCD file {}", path);
    }

    let mut layout = PointLayout::default();
    layout.add_attribute(
        attributes::POSITION_3D,
        pasture_core::layout::FieldAlignment::Default,
    );
    for field in &dynamic_fields {
        match field {
            DynFieldType::Classification => layout.add_attribute(
                attributes::CLASSIFICATION,
                pasture_core::layout::FieldAlignment::Default,
            ),
            DynFieldType::SourceID => layout.add_attribute(
                attributes::POINT_SOURCE_ID,
                pasture_core::layout::FieldAlignment::Default,
            ),
            DynFieldType::Intensity => layout.add_attribute(
                attributes::INTENSITY,
                pasture_core::layout::FieldAlignment::Default,
            ),
            DynFieldType::Color => layout.add_attribute(
                attributes::COLOR_RGB,
                pasture_core::layout::FieldAlignment::Default,
            ),
            DynFieldType::GpsTime => layout.add_attribute(
                attributes::GPS_TIME,
                pasture_core::layout::FieldAlignment::Default,
            ),
            DynFieldType::UserData => layout.add_attribute(
                attributes::USER_DATA,
                pasture_core::layout::FieldAlignment::Default,
            ),
            DynFieldType::ReturnNumber => layout.add_attribute(
                attributes::RETURN_NUMBER,
                pasture_core::layout::FieldAlignment::Default,
            ),
            DynFieldType::NumberOfReturns => layout.add_attribute(
                attributes::NUMBER_OF_RETURNS,
                pasture_core::layout::FieldAlignment::Default,
            ),
            DynFieldType::Skip => {}
        }
    }

    let num_points = points.len();
    let mut buffer = VectorBuffer::with_capacity(num_points, layout);
    buffer.resize(num_points); // Allocate space for all points

    let pos_attr = buffer
        .point_layout()
        .get_attribute(&attributes::POSITION_3D)
        .unwrap()
        .attribute_definition()
        .clone();
    let attrs: Vec<_> = dynamic_fields
        .iter()
        .map(|field| match field {
            DynFieldType::Classification => Some(
                buffer
                    .point_layout()
                    .get_attribute(&attributes::CLASSIFICATION)
                    .unwrap()
                    .attribute_definition()
                    .clone(),
            ),
            DynFieldType::SourceID => Some(
                buffer
                    .point_layout()
                    .get_attribute(&attributes::POINT_SOURCE_ID)
                    .unwrap()
                    .attribute_definition()
                    .clone(),
            ),
            DynFieldType::Intensity => Some(
                buffer
                    .point_layout()
                    .get_attribute(&attributes::INTENSITY)
                    .unwrap()
                    .attribute_definition()
                    .clone(),
            ),
            DynFieldType::Color => Some(
                buffer
                    .point_layout()
                    .get_attribute(&attributes::COLOR_RGB)
                    .unwrap()
                    .attribute_definition()
                    .clone(),
            ),
            DynFieldType::GpsTime => Some(
                buffer
                    .point_layout()
                    .get_attribute(&attributes::GPS_TIME)
                    .unwrap()
                    .attribute_definition()
                    .clone(),
            ),
            DynFieldType::UserData => Some(
                buffer
                    .point_layout()
                    .get_attribute(&attributes::USER_DATA)
                    .unwrap()
                    .attribute_definition()
                    .clone(),
            ),
            DynFieldType::ReturnNumber => Some(
                buffer
                    .point_layout()
                    .get_attribute(&attributes::RETURN_NUMBER)
                    .unwrap()
                    .attribute_definition()
                    .clone(),
            ),
            DynFieldType::NumberOfReturns => Some(
                buffer
                    .point_layout()
                    .get_attribute(&attributes::NUMBER_OF_RETURNS)
                    .unwrap()
                    .attribute_definition()
                    .clone(),
            ),
            DynFieldType::Skip => None,
        })
        .collect();

    let n_dyn_fields = dynamic_fields.len();
    for (i, p) in points.into_iter().enumerate() {
        let xyz: [f32; 3] = p.to_xyz().context("Unable to find xyz in PCD schema.")?;
        let pos_data = [
            xyz[0] as f64 * factor,
            xyz[1] as f64 * factor,
            xyz[2] as f64 * factor,
        ];
        let other_fields = &p.0[3..];
        let other_fields_count = other_fields.len();
        if n_dyn_fields > 0 && other_fields_count != n_dyn_fields {
            bail!(
                "Expected there to be {} dynamic fields in PCD file {}, but found {}",
                dynamic_fields.len(),
                path,
                other_fields.len()
            )
        }

        unsafe {
            buffer.set_attribute(&pos_attr, i, bytemuck::cast_slice(&pos_data));
        }

        for (dyn_field, field) in attrs.iter().zip(other_fields) {
            match dyn_field {
                Some(attr_def) => match attr_def {
                    def if *def == attributes::CLASSIFICATION => {
                        let cls = dyn_field_as_u8(field)?;
                        let data = [cls];
                        unsafe { buffer.set_attribute(attr_def, i, bytemuck::cast_slice(&data)) }
                    }
                    def if *def == attributes::POINT_SOURCE_ID => {
                        let source_id = dyn_field_as_u16(field)?;
                        let data = [source_id];
                        unsafe { buffer.set_attribute(attr_def, i, bytemuck::cast_slice(&data)) }
                    }
                    def if *def == attributes::INTENSITY => {
                        let intensity = dyn_field_as_u16(field)?;
                        let data = [intensity];
                        unsafe { buffer.set_attribute(attr_def, i, bytemuck::cast_slice(&data)) }
                    }
                    def if *def == attributes::COLOR_RGB => {
                        // Handle both packed RGB and separate fields if possible?
                        // For now, assume packed U32 if it's one field.
                        if let Some(rgb) = field.to_value::<u32>() {
                            let r = (((rgb >> 16) & 0xFF) as u16) << 8;
                            let g = (((rgb >> 8) & 0xFF) as u16) << 8;
                            let b = ((rgb & 0xFF) as u16) << 8;
                            let data = [r, g, b];
                            unsafe { buffer.set_attribute(attr_def, i, bytemuck::cast_slice(&data)) }
                        } else if let Some(rgb_f) = field.to_value::<f32>() {
                            let rgb = rgb_f.to_bits();
                            let r = (((rgb >> 16) & 0xFF) as u16) << 8;
                            let g = (((rgb >> 8) & 0xFF) as u16) << 8;
                            let b = ((rgb & 0xFF) as u16) << 8;
                            let data = [r, g, b];
                            unsafe { buffer.set_attribute(attr_def, i, bytemuck::cast_slice(&data)) }
                        }
                    }
                    def if *def == attributes::GPS_TIME => {
                        let gt = dyn_field_as_f64(field)?;
                        let data = [gt];
                        unsafe { buffer.set_attribute(attr_def, i, bytemuck::cast_slice(&data)) }
                    }
                    def if *def == attributes::USER_DATA => {
                        let ud = dyn_field_as_u8(field)?;
                        let data = [ud];
                        unsafe { buffer.set_attribute(attr_def, i, bytemuck::cast_slice(&data)) }
                    }
                    def if *def == attributes::RETURN_NUMBER => {
                        let rn = dyn_field_as_u8(field)?;
                        let data = [rn];
                        unsafe { buffer.set_attribute(attr_def, i, bytemuck::cast_slice(&data)) }
                    }
                    def if *def == attributes::NUMBER_OF_RETURNS => {
                        let nor = dyn_field_as_u8(field)?;
                        let data = [nor];
                        unsafe { buffer.set_attribute(attr_def, i, bytemuck::cast_slice(&data)) }
                    }
                    _ => {}
                },
                None => {}
            }
        }
    }

    Ok(buffer)
}

fn get_prop_f64(p: &Property) -> f64 {
    match p {
        Property::Float(f) => *f as f64,
        Property::Double(d) => *d,
        Property::UChar(u) => *u as f64,
        Property::Char(c) => *c as f64,
        Property::UShort(u) => *u as f64,
        Property::Short(s) => *s as f64,
        Property::UInt(u) => *u as f64,
        Property::Int(i) => *i as f64,
        _ => 0.0,
    }
}

fn get_prop_u16(p: &Property) -> u16 {
    match p {
        Property::UShort(u) => *u,
        Property::UChar(u) => (*u as u16) * 257,
        Property::UInt(u) => *u as u16,
        Property::Int(i) => *i as u16,
        Property::Float(f) => (*f * 65535.0) as u16,
        Property::Double(d) => (*d * 65535.0) as u16,
        _ => 0,
    }
}

fn get_prop_u8(p: &Property) -> u8 {
    match p {
        Property::UChar(u) => *u,
        Property::Char(c) => *c as u8,
        Property::UShort(u) => (*u >> 8) as u8,
        Property::Short(s) => (*s >> 8) as u8,
        Property::UInt(u) => *u as u8,
        Property::Int(i) => *i as u8,
        _ => 0,
    }
}

/// Read a .ply file into a VectorBuffer.
pub fn read_ply_file(path: &str, factor: f64) -> Result<VectorBuffer> {
    let mut f = BufReader::new(File::open(path)?);
    let p = ply_rs::parser::Parser::<ply_rs::ply::DefaultElement>::new();
    let ply = p.read_ply(&mut f).context("Failed to read PLY file")?;

    let vertices = ply.payload.get("vertex").context("No 'vertex' element in PLY file")?;
    if vertices.is_empty() {
        bail!("No points found in the PLY file {}", path);
    }

    let mut layout = PointLayout::default();
    let first_v = &vertices[0];
    
    if !first_v.contains_key("x") || !first_v.contains_key("y") || !first_v.contains_key("z") {
        bail!("PLY file must have x, y, and z properties in the 'vertex' element");
    }
    
    layout.add_attribute(attributes::POSITION_3D, pasture_core::layout::FieldAlignment::Default);
    
    if first_v.contains_key("red") && first_v.contains_key("green") && first_v.contains_key("blue") {
        layout.add_attribute(attributes::COLOR_RGB, pasture_core::layout::FieldAlignment::Default);
    }
    
    if first_v.contains_key("intensity") {
        layout.add_attribute(attributes::INTENSITY, pasture_core::layout::FieldAlignment::Default);
    }
    
    if first_v.contains_key("classification") || first_v.contains_key("scalar_Classification") {
        layout.add_attribute(attributes::CLASSIFICATION, pasture_core::layout::FieldAlignment::Default);
    }
    
    if first_v.contains_key("user_data") || first_v.contains_key("scalar_UserData") {
        layout.add_attribute(attributes::USER_DATA, pasture_core::layout::FieldAlignment::Default);
    }
    
    if first_v.contains_key("gps_time") || first_v.contains_key("scalar_GpsTime") {
        layout.add_attribute(attributes::GPS_TIME, pasture_core::layout::FieldAlignment::Default);
    }

    if first_v.contains_key("number_of_returns") {
        layout.add_attribute(attributes::NUMBER_OF_RETURNS, pasture_core::layout::FieldAlignment::Default);
    }
    
    if first_v.contains_key("return_number") {
        layout.add_attribute(attributes::RETURN_NUMBER, pasture_core::layout::FieldAlignment::Default);
    }

    if first_v.contains_key("point_source_id") {
        layout.add_attribute(attributes::POINT_SOURCE_ID, pasture_core::layout::FieldAlignment::Default);
    }

    let num_points = vertices.len();
    let mut buffer = VectorBuffer::with_capacity(num_points, layout);
    buffer.resize(num_points);
    
    let layout_copy = buffer.point_layout().clone();
    
    for (i, v) in vertices.iter().enumerate() {
        if let Some(attr) = layout_copy.get_attribute(&attributes::POSITION_3D) {
            let x = get_prop_f64(v.get("x").unwrap()) * factor;
            let y = get_prop_f64(v.get("y").unwrap()) * factor;
            let z = get_prop_f64(v.get("z").unwrap()) * factor;
            let pos_data = [x, y, z];
            unsafe { buffer.set_attribute(attr.attribute_definition(), i, bytemuck::cast_slice(&pos_data)) };
        }
        
        if let Some(attr) = layout_copy.get_attribute(&attributes::COLOR_RGB) {
            let r = get_prop_u16(v.get("red").unwrap());
            let g = get_prop_u16(v.get("green").unwrap());
            let b = get_prop_u16(v.get("blue").unwrap());
            let color_data = [r, g, b];
            unsafe { buffer.set_attribute(attr.attribute_definition(), i, bytemuck::cast_slice(&color_data)) };
        }
        
        if let Some(attr) = layout_copy.get_attribute(&attributes::INTENSITY) {
            let intensity = get_prop_u16(v.get("intensity").unwrap());
            let intensity_data = [intensity];
            unsafe { buffer.set_attribute(attr.attribute_definition(), i, bytemuck::cast_slice(&intensity_data)) };
        }
        
        if let Some(attr) = layout_copy.get_attribute(&attributes::CLASSIFICATION) {
            let cls = v.get("classification").or_else(|| v.get("scalar_Classification")).map(get_prop_u8).unwrap_or(0);
            let cls_data = [cls];
            unsafe { buffer.set_attribute(attr.attribute_definition(), i, bytemuck::cast_slice(&cls_data)) };
        }
        
        if let Some(attr) = layout_copy.get_attribute(&attributes::USER_DATA) {
            let ud = v.get("user_data").or_else(|| v.get("scalar_UserData")).map(get_prop_u8).unwrap_or(0);
            let ud_data = [ud];
            unsafe { buffer.set_attribute(attr.attribute_definition(), i, bytemuck::cast_slice(&ud_data)) };
        }
        
        if let Some(attr) = layout_copy.get_attribute(&attributes::GPS_TIME) {
            let gt = v.get("gps_time").or_else(|| v.get("scalar_GpsTime")).map(get_prop_f64).unwrap_or(0.0);
            let gt_data = [gt];
            unsafe { buffer.set_attribute(attr.attribute_definition(), i, bytemuck::cast_slice(&gt_data)) };
        }

        if let Some(attr) = layout_copy.get_attribute(&attributes::POINT_SOURCE_ID) {
            let sid = get_prop_u16(v.get("point_source_id").unwrap());
            let sid_data = [sid];
            unsafe { buffer.set_attribute(attr.attribute_definition(), i, bytemuck::cast_slice(&sid_data)) };
        }

        if let Some(attr) = layout_copy.get_attribute(&attributes::NUMBER_OF_RETURNS) {
            let nor = get_prop_u8(v.get("number_of_returns").unwrap());
            let nor_data = [nor];
            unsafe { buffer.set_attribute(attr.attribute_definition(), i, bytemuck::cast_slice(&nor_data)) };
        }

        if let Some(attr) = layout_copy.get_attribute(&attributes::RETURN_NUMBER) {
            let rn = get_prop_u8(v.get("return_number").unwrap());
            let rn_data = [rn];
            unsafe { buffer.set_attribute(attr.attribute_definition(), i, bytemuck::cast_slice(&rn_data)) };
        }
    }

    Ok(buffer)
}

/// Write a VectorBuffer to a .ply file.
pub fn write_ply_file(buffer: &VectorBuffer, path: &str) -> Result<()> {
    let mut ply = ply_rs::ply::Ply::<ply_rs::ply::DefaultElement>::new();
    ply.header.encoding = ply_rs::ply::Encoding::BinaryLittleEndian;
    
    let mut vertex_element = ply_rs::ply::ElementDef::new("vertex".to_string());
    let layout = buffer.point_layout();
    
    if layout.has_attribute(&attributes::POSITION_3D) {
        vertex_element.properties.add(ply_rs::ply::PropertyDef::new("x".to_string(), PropertyType::Scalar(ScalarType::Double)));
        vertex_element.properties.add(ply_rs::ply::PropertyDef::new("y".to_string(), PropertyType::Scalar(ScalarType::Double)));
        vertex_element.properties.add(ply_rs::ply::PropertyDef::new("z".to_string(), PropertyType::Scalar(ScalarType::Double)));
    }
    
    if layout.has_attribute(&attributes::COLOR_RGB) {
        vertex_element.properties.add(ply_rs::ply::PropertyDef::new("red".to_string(), PropertyType::Scalar(ScalarType::UChar)));
        vertex_element.properties.add(ply_rs::ply::PropertyDef::new("green".to_string(), PropertyType::Scalar(ScalarType::UChar)));
        vertex_element.properties.add(ply_rs::ply::PropertyDef::new("blue".to_string(), PropertyType::Scalar(ScalarType::UChar)));
    }
    
    if layout.has_attribute(&attributes::INTENSITY) {
        vertex_element.properties.add(ply_rs::ply::PropertyDef::new("intensity".to_string(), PropertyType::Scalar(ScalarType::UShort)));
    }
    
    if layout.has_attribute(&attributes::CLASSIFICATION) {
        vertex_element.properties.add(ply_rs::ply::PropertyDef::new("classification".to_string(), PropertyType::Scalar(ScalarType::UChar)));
    }
    
    if layout.has_attribute(&attributes::GPS_TIME) {
        vertex_element.properties.add(ply_rs::ply::PropertyDef::new("gps_time".to_string(), PropertyType::Scalar(ScalarType::Double)));
    }
    
    if layout.has_attribute(&attributes::POINT_SOURCE_ID) {
        vertex_element.properties.add(ply_rs::ply::PropertyDef::new("point_source_id".to_string(), PropertyType::Scalar(ScalarType::UShort)));
    }

    if layout.has_attribute(&attributes::USER_DATA) {
        vertex_element.properties.add(ply_rs::ply::PropertyDef::new("user_data".to_string(), PropertyType::Scalar(ScalarType::UChar)));
    }

    if layout.has_attribute(&attributes::NUMBER_OF_RETURNS) {
        vertex_element.properties.add(ply_rs::ply::PropertyDef::new("number_of_returns".to_string(), PropertyType::Scalar(ScalarType::UChar)));
    }

    if layout.has_attribute(&attributes::RETURN_NUMBER) {
        vertex_element.properties.add(ply_rs::ply::PropertyDef::new("return_number".to_string(), PropertyType::Scalar(ScalarType::UChar)));
    }
    
    ply.header.elements.add(vertex_element);
    
    let mut vertices = Vec::with_capacity(buffer.len());
    
    for i in 0..buffer.len() {
        let mut element = ply_rs::ply::DefaultElement::new();
        
        if layout.has_attribute(&attributes::POSITION_3D) {
            let pos = buffer.view_attribute::<Vector3<f64>>(&attributes::POSITION_3D).at(i);
            element.insert("x".to_string(), Property::Double(pos.x));
            element.insert("y".to_string(), Property::Double(pos.y));
            element.insert("z".to_string(), Property::Double(pos.z));
        }
        
        if layout.has_attribute(&attributes::COLOR_RGB) {
            let color = buffer.view_attribute::<Vector3<u16>>(&attributes::COLOR_RGB).at(i);
            element.insert("red".to_string(), Property::UChar((color.x >> 8) as u8));
            element.insert("green".to_string(), Property::UChar((color.y >> 8) as u8));
            element.insert("blue".to_string(), Property::UChar((color.z >> 8) as u8));
        }
        
        if layout.has_attribute(&attributes::INTENSITY) {
            let intensity = buffer.view_attribute::<u16>(&attributes::INTENSITY).at(i);
            element.insert("intensity".to_string(), Property::UShort(intensity));
        }
        
        if layout.has_attribute(&attributes::CLASSIFICATION) {
            let cls = buffer.view_attribute::<u8>(&attributes::CLASSIFICATION).at(i);
            element.insert("classification".to_string(), Property::UChar(cls));
        }
        
        if layout.has_attribute(&attributes::GPS_TIME) {
            let gt = buffer.view_attribute::<f64>(&attributes::GPS_TIME).at(i);
            element.insert("gps_time".to_string(), Property::Double(gt));
        }

        if layout.has_attribute(&attributes::POINT_SOURCE_ID) {
            let sid = buffer.view_attribute::<u16>(&attributes::POINT_SOURCE_ID).at(i);
            element.insert("point_source_id".to_string(), Property::UShort(sid));
        }

        if layout.has_attribute(&attributes::USER_DATA) {
            let ud = buffer.view_attribute::<u8>(&attributes::USER_DATA).at(i);
            element.insert("user_data".to_string(), Property::UChar(ud));
        }

        if layout.has_attribute(&attributes::NUMBER_OF_RETURNS) {
            let nor = buffer.view_attribute::<u8>(&attributes::NUMBER_OF_RETURNS).at(i);
            element.insert("number_of_returns".to_string(), Property::UChar(nor));
        }

        if layout.has_attribute(&attributes::RETURN_NUMBER) {
            let rn = buffer.view_attribute::<u8>(&attributes::RETURN_NUMBER).at(i);
            element.insert("return_number".to_string(), Property::UChar(rn));
        }
        
        vertices.push(element);
    }
    
    ply.payload.insert("vertex".to_string(), vertices);
    
    let mut f = BufWriter::new(File::create(path)?);
    let writer = ply_rs::writer::Writer::new();
    writer.write_ply(&mut f, &mut ply).context("Failed to write PLY file")?;
    
    Ok(())
}

/// Write a VectorBuffer to a .pcd file.
pub fn write_pcd_file(buffer: &VectorBuffer, path: &str) -> Result<()> {
    let layout = buffer.point_layout();
    let mut schema_fields = Vec::new();
    
    if layout.has_attribute(&attributes::POSITION_3D) {
        schema_fields.push(("x", ValueKind::F32, 1));
        schema_fields.push(("y", ValueKind::F32, 1));
        schema_fields.push(("z", ValueKind::F32, 1));
    }
    
    if layout.has_attribute(&attributes::INTENSITY) {
        schema_fields.push(("intensity", ValueKind::U16, 1));
    }
    
    if layout.has_attribute(&attributes::COLOR_RGB) {
        schema_fields.push(("rgb", ValueKind::U32, 1));
    }
    
    if layout.has_attribute(&attributes::CLASSIFICATION) {
        schema_fields.push(("label", ValueKind::U8, 1));
    }

    if layout.has_attribute(&attributes::GPS_TIME) {
        schema_fields.push(("gps_time", ValueKind::F64, 1));
    }

    if layout.has_attribute(&attributes::POINT_SOURCE_ID) {
        schema_fields.push(("point_source_id", ValueKind::U16, 1));
    }

    if layout.has_attribute(&attributes::USER_DATA) {
        schema_fields.push(("user_data", ValueKind::U8, 1));
    }

    if layout.has_attribute(&attributes::NUMBER_OF_RETURNS) {
        schema_fields.push(("number_of_returns", ValueKind::U8, 1));
    }

    if layout.has_attribute(&attributes::RETURN_NUMBER) {
        schema_fields.push(("return_number", ValueKind::U8, 1));
    }

    let writer_init = WriterInit {
        width: buffer.len() as u64,
        height: 1,
        viewpoint: Default::default(),
        data_kind: DataKind::Binary,
        schema: Some(Schema::from_iter(schema_fields)),
        version: Some(String::from("0.7")),
    };
    
    let mut writer = writer_init.create(path).context("Failed to create PCD writer")?;
    
    for i in 0..buffer.len() {
        let mut fields = Vec::new();
        
        if layout.has_attribute(&attributes::POSITION_3D) {
            let pos = buffer.view_attribute::<Vector3<f64>>(&attributes::POSITION_3D).at(i);
            fields.push(Field::F32(vec![pos.x as f32]));
            fields.push(Field::F32(vec![pos.y as f32]));
            fields.push(Field::F32(vec![pos.z as f32]));
        }
        
        if layout.has_attribute(&attributes::INTENSITY) {
            let intensity = buffer.view_attribute::<u16>(&attributes::INTENSITY).at(i);
            fields.push(Field::U16(vec![intensity]));
        }
        
        if layout.has_attribute(&attributes::COLOR_RGB) {
            let color = buffer.view_attribute::<Vector3<u16>>(&attributes::COLOR_RGB).at(i);
            let r = (color.x >> 8) as u32;
            let g = (color.y >> 8) as u32;
            let b = (color.z >> 8) as u32;
            let rgb = (r << 16) | (g << 8) | b;
            fields.push(Field::U32(vec![rgb]));
        }
        
        if layout.has_attribute(&attributes::CLASSIFICATION) {
            let cls = buffer.view_attribute::<u8>(&attributes::CLASSIFICATION).at(i);
            fields.push(Field::U8(vec![cls]));
        }

        if layout.has_attribute(&attributes::GPS_TIME) {
            let gt = buffer.view_attribute::<f64>(&attributes::GPS_TIME).at(i);
            fields.push(Field::F64(vec![gt]));
        }

        if layout.has_attribute(&attributes::POINT_SOURCE_ID) {
            let sid = buffer.view_attribute::<u16>(&attributes::POINT_SOURCE_ID).at(i);
            fields.push(Field::U16(vec![sid]));
        }

        if layout.has_attribute(&attributes::USER_DATA) {
            let ud = buffer.view_attribute::<u8>(&attributes::USER_DATA).at(i);
            fields.push(Field::U8(vec![ud]));
        }

        if layout.has_attribute(&attributes::NUMBER_OF_RETURNS) {
            let nor = buffer.view_attribute::<u8>(&attributes::NUMBER_OF_RETURNS).at(i);
            fields.push(Field::U8(vec![nor]));
        }

        if layout.has_attribute(&attributes::RETURN_NUMBER) {
            let rn = buffer.view_attribute::<u8>(&attributes::RETURN_NUMBER).at(i);
            fields.push(Field::U8(vec![rn]));
        }
        
        writer.push(&DynRecord(fields)).context("Failed to push point to PCD writer")?;
    }
    
    writer.finish().context("Failed to finish PCD writer")?;
    Ok(())
}

/// Gather pointcloud paths (.las/.laz/.pcd/.ply).
pub fn gather_pointcloud_paths(input: &str, recursive: bool) -> Result<Vec<String>> {
    let mut paths = Vec::new();
    let input_path = Path::new(input);

    if input_path.is_file() {
        let ext = extension(input);
        if is_supported_extension(&ext) {
            paths.push(input.to_string());
        }
    } else if input_path.is_dir() {
        if recursive {
            for entry in WalkDir::new(input_path).into_iter().filter_map(Result::ok) {
                if entry.file_type().is_file() {
                    let p = entry.path().to_string_lossy().to_string();
                    if is_supported_extension(&extension(&p)) {
                        paths.push(p);
                    }
                }
            }
        } else {
            for entry in std::fs::read_dir(input_path)? {
                let e = entry?;
                let p = e.path();
                if p.is_file() {
                    let ps = p.to_string_lossy().to_string();
                    if is_supported_extension(&extension(&ps)) {
                        paths.push(ps);
                    }
                }
            }
        }
    }

    Ok(paths)
}
