/**
 * @file PhoXiFrame.h
 * @brief Lightweight view into a single frame delivered by the PhoXi C API.
 */
#ifndef PHOXI_CAMERA_PHOXIFRAME_H
#define PHOXI_CAMERA_PHOXIFRAME_H

#include "phoxi/phoxi_c_api.h"

namespace phoxi_camera
{

/**
 * @brief Typed view into a single frame delivered by the PhoXi API.
 *
 * Each field is a non-owning pointer into the API-managed buffer.
 * All pointers are valid only for the duration of the `GetFrameCallback` invocation.
 * A null pointer means the corresponding data component was not enabled or not available.
 *
 * The API TEXTURE record is split here into `texture` (FLOAT_32F grayscale) and
 * `textureRgb` (RGB_16); at most one of them is non-null per frame.
 */
struct PhoXiFrame
{
    const phoxi_frame_record_t* frameInfo = nullptr;      ///< JSON frame metadata and status (FRAME_INFO record).
    const phoxi_frame_record_t* pointCloud = nullptr;     ///< 3D point cloud; float[3] per pixel in mm.
    const phoxi_frame_record_t* normalMap = nullptr;      ///< Per-pixel surface normal vectors; float[3] per pixel.
    const phoxi_frame_record_t* depthMap = nullptr;       ///< Orthogonal distance from camera; float per pixel in mm.
    const phoxi_frame_record_t* confidenceMap = nullptr;  ///< Measurement confidence; float per pixel in [0, 1].
    const phoxi_frame_record_t* eventMap = nullptr;       ///< Time of measurement; float per pixel (MotionCam Camera mode only).
    const phoxi_frame_record_t* texture = nullptr;        ///< Grayscale texture; FLOAT_32F per pixel in [0, 2047].
    const phoxi_frame_record_t* textureRgb = nullptr;     ///< RGB texture; uint16_t[3] per pixel in [0, 1023].
    const phoxi_frame_record_t* colorCamera = nullptr;    ///< Color camera image; uint16_t[3] per pixel in [0, 1023].
};

}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_PHOXIFRAME_H
