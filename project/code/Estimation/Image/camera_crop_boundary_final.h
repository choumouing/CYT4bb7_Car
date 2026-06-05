#ifndef CAMERA_CROP_BOUNDARY_FINAL_H
#define CAMERA_CROP_BOUNDARY_FINAL_H

/*
 * Final crop boundaries in image pixel coordinates.
 * x is in [0, 187]. The area below the curve is cropped because image y grows
 * downward, so the crop test is y > boundary_y.
 */

static inline float camera_front_crop_boundary_y(float x)
{
    const float image_x = 94.0f - x;

    return 60.0f - (9.0f -
                    (0.003202929f * image_x) +
                    (0.003202929f * image_x * image_x));
}

static inline float camera_rear_crop_boundary_y(float x)
{
    return (-7.0f / 4371.0f) * x * x + (1309.0f / 4371.0f) * x + 32.0f;
}

static inline int camera_front_crop_below(float x, float y)
{
    return y > camera_front_crop_boundary_y(x);
}

static inline int camera_rear_crop_below(float x, float y)
{
    return y > camera_rear_crop_boundary_y(x);
}

static inline int camera_front_crop_outside(float x, float y)
{
    return camera_front_crop_below(x, y);
}

static inline int camera_rear_crop_outside(float x, float y)
{
    return camera_rear_crop_below(x, y);
}

#endif /* CAMERA_CROP_BOUNDARY_FINAL_H */
