LSH Vision
=============

## Description

LSH Vision uses LSH on raw pixel data to find close matches to hand picked example images for object classification. This is a research module aiming to see if we can achieve fast, robust detection of objects.

## Usage

LSH Vision can be used either instead of or as well as LUT based vision. Expect worse width based measurements as these will likely be more noisy under LSHvision. The primary use case is for finding posters around the edges of the field

## Consumes

* `messages::Image' - image from the camera to be checked

## Emits

* 'messages::ClassifiedObjects' - Object locations, bounding boxes, and labels

## Configuration

* size: the x and y size of the LSH box to scan with
* stride: the x and y increment in pixels to scan with
* images: labeled image files for use in the LSH classifier

## Dependencies

* Requires the camera module

