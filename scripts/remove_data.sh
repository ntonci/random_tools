#!/bin/bash

base_path=$1

if [ -d "$base_path/depth_image" ]; then
  cd $base_path/depth_image
  echo "depth_image\n"
  rm *.exr
fi

if [ -d "$base_path/depth_image_hw_rect" ]; then
  cd $base_path/depth_image_hw_rect
  echo "depth_image_hw_rect\n"
  rm *.exr
fi

if [ -d "$base_path/depth_image_hw_rect_raw" ]; then
  cd $base_path/depth_image_hw_rect_raw
  echo "depth_image_hw_rect_raw\n"
  rm *.tiff
fi

if [ -d "$base_path/depth_image_raw" ]; then
  cd $base_path/depth_image_raw
  echo "depth_image_raw\n"
  rm *.tiff
fi

if [ -d "$base_path/depth_image_rect" ]; then
  cd $base_path/depth_image_rect
  echo "depth_image_rect\n"
  rm *.exr
fi

if [ -d "$base_path/depth_image_rect_raw" ]; then
  cd $base_path/depth_image_rect_raw
  echo "depth_image_rect_raw\n"
  rm *.tiff
fi

if [ -d "$base_path/depth_image_reg" ]; then
  cd $base_path/depth_image_reg
  echo "depth_image_reg\n"
  rm *.exr
fi

if [ -d "$base_path/depth_image_reg_raw" ]; then
  cd $base_path/depth_image_reg_raw
  echo "depth_image_reg_raw\n"
  rm *.tiff
fi

if [ -d "$base_path/depth_image_sw_rect" ]; then
  cd $base_path/depth_image_sw_rect
  echo "depth_image_sw_rect\n"
  rm *.exr
fi

if [ -d "$base_path/depth_image_sw_rect_raw" ]; then
  cd $base_path/depth_image_sw_rect_raw
  echo "depth_image_sw_rect_raw\n"
  rm *.tiff
fi

if [ -d "$base_path/pc" ]; then
  cd $base_path/pc
  echo "pc\n"
  rm *.ply
fi

if [ -d "$base_path/pc_reg" ]; then
  cd $base_path/pc_reg
  echo "pc_reg\n"
  rm *.ply
fi

if [ -d "$base_path/rgb" ]; then
  cd $base_path/rgb
  echo "rgb\n"
  rm *.png
fi

if [ -d "$base_path/rgb_rect" ]; then
  cd $base_path/rgb_rect
  echo "rgb_rect\n"
  rm *.png
fi
