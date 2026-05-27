#ifndef PHOXI_CAMERA_PHOXIFRAME_H
#define PHOXI_CAMERA_PHOXIFRAME_H

#include "phoxi/phoxi_c_api.h"

namespace phoxi_camera
{

/**
 * Typed view into a single frame delivered by the PhoXi SDK.
 *
 * Each field is a pointer to the relevant record inside the SDK-owned buffer.
 * All pointers remain valid only for the duration of the frame callback.
 * The TEXTURE record is split into texture (FLOAT_32F) and textureRgb (RGB_16).
 */
struct PhoXiFrame
{
    const phoxi_frame_record_t* pointCloud = nullptr;
    const phoxi_frame_record_t* normalMap = nullptr;
    const phoxi_frame_record_t* depthMap = nullptr;
    const phoxi_frame_record_t* confidenceMap = nullptr;
    const phoxi_frame_record_t* eventMap = nullptr;
    const phoxi_frame_record_t* texture = nullptr;     // TEXTURE, FLOAT_32F (grayscale)
    const phoxi_frame_record_t* textureRgb = nullptr;  // TEXTURE, RGB_16
    const phoxi_frame_record_t* colorCamera = nullptr;
};

}  // namespace phoxi_camera

#endif  // PHOXI_CAMERA_PHOXIFRAME_H
