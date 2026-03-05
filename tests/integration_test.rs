mod common;

use common::{create_mock_pcd, create_mock_poses};
use pasture_core::containers::BorrowedBuffer;
use pcutil::{
    commands::pointcloud::{
        convert, merge, pointcloud_utils::read_pointcloud_file_to_buffer, summary, voxelize,
    },
    PointcloudConvertArgs, PointcloudMergeArgs, PointcloudSummaryArgs, PointcloudVoxelizeArgs,
};
use tempfile::tempdir;

#[test]
fn test_summary() {
    let dir = tempdir().unwrap();
    let pcd_path = dir.path().join("test.pcd");
    let pcd_path_str = pcd_path.to_str().unwrap();

    let points = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ];
    create_mock_pcd(pcd_path_str, &points, None).unwrap();

    let args = PointcloudSummaryArgs {
        input: vec![pcd_path_str.to_string()],
        dynamic_fields: vec![],
        factor: 1.0,
        recursive: false,
    };

    let result = summary::execute(args);
    assert!(result.is_ok());
}

#[test]
fn test_convert() {
    let dir = tempdir().unwrap();
    let input_path = dir.path().join("input.pcd");
    let output_path = dir.path().join("output.ply");
    let input_path_str = input_path.to_str().unwrap();
    let output_path_str = output_path.to_str().unwrap();

    let points = [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]];
    create_mock_pcd(input_path_str, &points, None).unwrap();

    let args = PointcloudConvertArgs {
        input: input_path_str.to_string(),
        output: output_path_str.to_string(),
        factor: 1.0,
        dynamic_fields: vec![],
        recursive: false,
        format: None,
    };

    let result = convert::execute(args);
    assert!(result.is_ok());
    assert!(output_path.exists());
}

#[test]
fn test_merge() {
    let dir = tempdir().unwrap();
    let frames_dir = dir.path().join("frames");
    std::fs::create_dir(&frames_dir).unwrap();
    
    let f1 = frames_dir.join("001.pcd");
    let f2 = frames_dir.join("002.pcd");
    let poses_path = dir.path().join("poses.npy");
    let output_path = dir.path().join("merged.pcd");

    let f1_str = f1.to_str().unwrap();
    let f2_str = f2.to_str().unwrap();
    let poses_path_str = poses_path.to_str().unwrap();
    let output_path_str = output_path.to_str().unwrap();
    let frames_dir_str = frames_dir.to_str().unwrap();

    create_mock_pcd(f1_str, &[[0.0, 0.0, 0.0]], None).unwrap();
    create_mock_pcd(f2_str, &[[1.0, 1.0, 1.0]], None).unwrap();
    create_mock_poses(poses_path_str, 2).unwrap();

    let args = PointcloudMergeArgs {
        input: frames_dir_str.to_string(),
        output: output_path_str.to_string(),
        poses: Some(poses_path_str.to_string()),
        factor: 1.0,
        voxel_size: None,
        dynamic_fields: vec![],
        recursive: false,
    };

    let result = merge::execute(args);
    assert!(result.is_ok());
    assert!(output_path.exists());

    let buffer = read_pointcloud_file_to_buffer(output_path_str, 1.0, vec![]).unwrap();
    assert_eq!(buffer.len(), 2);
}

#[test]
fn test_voxelize() {
    let dir = tempdir().unwrap();
    let input_path = dir.path().join("input.pcd");
    let output_path = dir.path().join("output.pcd");
    let input_path_str = input_path.to_str().unwrap();
    let output_path_str = output_path.to_str().unwrap();

    // Two points very close to each other
    let points = [[0.0, 0.0, 0.0], [0.01, 0.01, 0.01]];
    create_mock_pcd(input_path_str, &points, None).unwrap();

    let args = PointcloudVoxelizeArgs {
        input: input_path_str.to_string(),
        output: output_path_str.to_string(),
        voxel_size: 1.0, // Should merge them
        factor: 1.0,
        dynamic_fields: vec![],
        recursive: false,
        format: None,
    };

    let result = voxelize::execute(args);
    assert!(result.is_ok());
    
    let buffer = read_pointcloud_file_to_buffer(output_path_str, 1.0, vec![]).unwrap();
    assert_eq!(buffer.len(), 1);
}
