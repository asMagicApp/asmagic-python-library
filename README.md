# asmagic

> A Python library for receiving AR sensor data streams from asMagic iOS App via ZeroMQ.

Working with asMagic App:

<a href="https://apps.apple.com/cn/app/asmagic/id6661033548">
  <img src="https://tools.applemediaservices.com/api/badges/download-on-the-app-store/black/en-us?size=250x83" alt="Download on the App Store" height="40">
</a>

Supports real-time streaming of:

-   6DOF camera pose (position + orientation)
-   RGB camera images
-   Depth maps
-   Camera intrinsics
-   Device velocity

## Install

```bash
pip install asmagic
```

## Quick Start

```python
from asmagic import ARDataSubscriber

# Create subscriber with your iPhone's IP address
sub = ARDataSubscriber("192.168.1.100")

try:
    # Continuous data streaming
    for data in sub:
        # All sensor data in one frame
        print(f"Timestamp: {data.timestamp}")
        print(f"Velocity: {data.velocity}")
        print(f"Local Pose: {data.local_pose}")
        print(f"Global Pose: {data.global_pose}")
        print(f"Camera Intrinsics: {data.camera_intrinsics}")

        # Access image data
        if data.has_color_image:
            # Color: bytes(jpeg format) or array
            color_bytes = data.color_bytes
            color_array = data.color_array  # or shortcut: data.color
            print(f"Color: {len(color_bytes)} bytes, array shape: {color_array.shape}")

        if data.has_depth_image:
            # Depth: Numpy array
            depth = data.depth_array  # or shortcut: data.depth
            print(f"Depth: {depth.shape}")

except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    sub.close()
```

## Usage Examples

### Example 1: Continuous Data Reading

```python
from asmagic import ARDataSubscriber

# Connect to iPhone
sub = ARDataSubscriber("192.168.1.100")

try:
    for data in sub:
        # Print all available data
        print(f"\n--- Frame at {data.timestamp} ---")
        print(f"Velocity: {data.velocity}")
        print(f"Local Pose: {data.local_pose}")
        print(f"Global Pose: {data.global_pose}")
        print(f"Camera Intrinsics: {data.camera_intrinsics}")

        if data.has_depth_image:
            depth = data.depth  # or data.depth_array
            print(f"Depth shape: {depth.shape}, min: {depth.min()}, max: {depth.max()}")

except KeyboardInterrupt:
    print("\nStopped")
finally:
    sub.close()
```

### Example 2: Display Color and Depth Images

```python
from asmagic import ARDataSubscriber
import cv2

# Connect to iPhone
sub = ARDataSubscriber("192.168.1.100")

try:
    for data in sub:
        # Display both images side by side
        data.show_images()

        # Or display individually:
        # data.show_color()  # RGB image
        # data.show_depth()  # Depth map with colormap

        # Press ESC to exit
        if cv2.waitKey(1) == 27:
            break

except KeyboardInterrupt:
    print("\nStopped")
finally:
    sub.close()
    cv2.destroyAllWindows()
```

### Example 3: Process Image Data

```python
from asmagic import ARDataSubscriber
import numpy as np

sub = ARDataSubscriber("192.168.1.100")

try:
    for data in sub:

        # Color image: process as numpy array
        if data.has_color_image:
            color = data.color_array  # RGB numpy array
            print(f"Color shape: {color.shape}")

        # Depth image: always as numpy array
        if data.has_depth_image:
            depth = data.depth_array  # uint16 numpy array
            depth_meters = depth.astype(np.float32) / 10000.0
            print(f"Depth range: {depth_meters.min():.2f}m - {depth_meters.max():.2f}m")

except KeyboardInterrupt:
    pass
finally:
    sub.close()
```

### Example 4: Using Pose Data

```python
from asmagic import ARDataSubscriber
import numpy as np

sub = ARDataSubscriber("192.168.1.100")

try:
    for data in sub:
        # Extract position from pose (first 3 elements)
        position = data.local_pose[:3] # [tx, ty, tz]

        # Extract quaternion from pose (last 4 elements)
        quaternion = data.local_pose[3:]  # [qx, qy, qz, qw]

        print(f"Position (m): x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}")
        print(f"Quaternion: {quaternion}")
        print(f"Velocity (m/s): {data.velocity}")   # [vx, vy, vz]

except KeyboardInterrupt:
    pass
finally:
    sub.close()
```

### Example 5: Continuous Reading with `get()`

```python
from asmagic import ARDataSubscriber

sub = ARDataSubscriber("192.168.1.100")

try:
    while True:
        data = sub.get()
        if data:
            print(f"Timestamp: {data.timestamp}")
            print(f"Velocity: {data.velocity}")
            print(f"Local Pose: {data.local_pose}")
            print(f"Global Pose: {data.global_pose}")
            print(f"Camera Intrinsics: {data.camera_intrinsics}")

            # Access images
            if data.has_color_image:
                print(f"Color bytes: {len(data.color_bytes)} bytes")
                print(f"Color array: {data.color_array.shape}")

            if data.has_depth_image:
                print(f"Depth array: {data.depth_array.shape}")

except KeyboardInterrupt:
    print("\nStopped")
finally:
    sub.close()
```

> **Note**: Example 5 (`while True` with `get()`) and Example 1 (`for data in sub:`) achieve the same goal of continuous data reading. 

### Example 6: Get Individual Data Fields

```python
from asmagic import ARDataSubscriber
import numpy as np

sub = ARDataSubscriber("192.168.1.100")

try:
    while True:
        # Get only specific data fields
        timestamp = sub.get_timestamp()
        print(f"Timestamp: {timestamp}")
       
except KeyboardInterrupt:
    print("\nStopped")
finally:
    sub.close()
```

> **Note**: Use `get_*()` methods when you only need specific data fields. This is more efficient than receiving the full frame.

## Data Format Reference

### Frame Data Fields

Each `data` object contains the following fields:

| Field               | Type         | Shape   | Unit    | Description                                                             |
| ------------------- | ------------ | ------- | ------- | ----------------------------------------------------------------------- |
| `timestamp`         | `float`      | scalar  | seconds | Unix timestamp (seconds since 1970-01-01)                               |
| `local_pose`        | `np.ndarray` | (7,)    | m, quat | Camera pose in device coordinate: $[t_x, t_y, t_z, q_x, q_y, q_z, q_w]$ |
| `global_pose`       | `np.ndarray` | (7,)    | m, quat | Camera pose in world coordinate: $[t_x, t_y, t_z, q_x, q_y, q_z, q_w]$  |
| `velocity`          | `np.ndarray` | (3,)    | m/s     | Linear velocity: $[v_x, v_y, v_z]$                                      |
| `camera_intrinsics` | `np.ndarray` | (9,)    | pixels  | 3×3 matrix (flattened): $[f_x, 0, 0, 0, f_y, 0, c_x, c_y, 1]$           |
| `color_bytes`       | `bytes`      | -       | -       | JPEG image bytes (640×480, quality=0.8)                                 |
| `color_array`       | `np.ndarray` | (H,W,3) | -       | Decoded RGB image as numpy array                                        |
| `depth_array`       | `np.ndarray` | (H,W)   | 10e-4 m | Depth image as numpy array (uint16)                                     |
| `depth_width`       | `int`        | scalar  | pixels  | Depth image width (256 width)                                           |
| `depth_height`      | `int`        | scalar  | pixels  | Depth image height (192 height)                                         |

### Coordinate System

**Pose Format**: $[t_x, t_y, t_z, q_x, q_y, q_z, q_w]$

-   **Translation** (first 3 values): Position in meters
    -   $t_x, t_y, t_z$: X, Y, Z coordinates
-   **Rotation** (last 4 values): Orientation as quaternion
    -   $q_x, q_y, q_z$: Imaginary part
    -   $q_w$: Real part
    -   Normalized: $q_x^2 + q_y^2 + q_z^2 + q_w^2 = 1$

**Coordinate Frame**:

-   **X-axis**: Right
-   **Y-axis**: Up
-   **Z-axis**: Forward

**Difference between local_pose and global_pose**:

-   `local_pose`: Position relative to device starting point
-   `global_pose`: Position in shared world coordinate (when multiple devices collaborate)

> **Note**: When using a single device, `local_pose` and `global_pose` are identical

### Velocity

**Format**: $[v_x, v_y, v_z]$

-   Calculated as: $\vec{v} = \frac{\Delta \vec{p}}{\Delta t}$
-   Unit: meters per second (m/s)
-   In the same coordinate frame as pose

### Camera Intrinsics

**Format**: $[f_x, 0, 0, 0, f_y, 0, c_x, c_y, 1]$

Represents 3×3 matrix:

$$
K = \begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
$$

Where:

-   $f_x, f_y$: Focal length in pixels
-   $c_x, c_y$: Principal point (optical center) in pixels

### Depth Image

-   **Format**: 16-bit unsigned integer (uint16)
-   **Unit**: $10^{-4}$ m (0.1 mm, scaled by 10000 from meters)
-   **Conversion to meters**: $d_{meters} = \frac{d_{raw}}{10000}$
-   **Range**: $0 \leq d_{raw} \leq 65535$ → $0$ to $6.5535$ m
-   **Access**: Use `data.depth` or `data.depth_array` to get numpy array

### Image Data Access

**Color Image**: Two formats available

-   `color_bytes`: JPEG compressed bytes - use for saving, forwarding, or storage
-   `color_array`: RGB numpy array (H×W×3) - use for processing with OpenCV, deep learning, etc.
-   `color`: Shortcut for `color_array`

**Depth Image**: Numpy array only

-   `depth_array`: uint16 numpy array (H×W) - ready for depth calculations
-   `depth`: Shortcut for `depth_array`

| Use Case                  | Use This      | Why                                    |
| ------------------------- | ------------- | -------------------------------------- |
| Save color to file        | `color_bytes` | No decoding needed, fastest            |
| Forward to network/API    | `color_bytes` | Keep compressed format, save bandwidth |
| Image processing (OpenCV) | `color_array` | Need numpy array for algorithms        |
| Deep learning             | `color_array` | Convert to tensors from numpy          |
| Depth calculation         | `depth_array` | Ready-to-use (H, W) array              |

## API Reference

### ARDataSubscriber

**Constructor:**

```python
ARDataSubscriber(ip, port=8000, hwm=1, conflate=True, verbose=False)
```

**Parameters:**

-   `ip` (str): iPhone's IP address
-   `port` (int): Port number (default: 8000)
-   `hwm` (int): High water mark (default: 1, keeps only latest message)
-   `conflate` (bool): Message conflation (default: True)
-   `verbose` (bool): Print connection info (default: False)

**Usage:**

```python
# Create subscriber
sub = ARDataSubscriber("192.168.1.100")

# Continuously receive data
for data in sub:
    print(data.timestamp)
    print(data.velocity)

# Close when done
sub.close()
```

**Main Methods:**

| Method                    | Returns                | Description                |
| ------------------------- | ---------------------- | -------------------------- |
| `get()`                   | `ARFrame` or `None`    | Get latest data frame      |
| `get_timestamp()`         | `float` or `None`      | Get timestamp only         |
| `get_velocity()`          | `np.ndarray` or `None` | Get velocity only          |
| `get_local_pose()`        | `np.ndarray` or `None` | Get local pose only        |
| `get_global_pose()`       | `np.ndarray` or `None` | Get global pose only       |
| `get_camera_intrinsics()` | `np.ndarray` or `None` | Get camera intrinsics only |
| `get_color_image()`       | `bytes` or `None`      | Get color image bytes only |
| `get_depth_image()`       | `np.ndarray` or `None` | Get depth array only       |
| `close()`                 | `None`                 | Close connection           |

**Note**:

-   The subscriber is iterable, so you can use `for data in sub:` to receive frames continuously.
-   All `get_*()` methods accept an optional `timeout` parameter (default: 1000ms).

### ARFrame

Data object returned by `get()` or when iterating.

**Properties** (see Data Format Reference above for details):

| Property            | Type         | Description                                       |
| ------------------- | ------------ | ------------------------------------------------- |
| `timestamp`         | `float`      | Unix timestamp in seconds                         |
| `velocity`          | `np.ndarray` | Velocity $[v_x, v_y, v_z]$ in m/s                 |
| `local_pose`        | `np.ndarray` | Local pose $[t_x, t_y, t_z, q_x, q_y, q_z, q_w]$  |
| `global_pose`       | `np.ndarray` | Global pose $[t_x, t_y, t_z, q_x, q_y, q_z, q_w]$ |
| `camera_intrinsics` | `np.ndarray` | Camera intrinsics (3×3 flattened)                 |
| **Color Image**     |              |                                                   |
| `color_bytes`       | `bytes`      | JPEG image bytes (for saving/forwarding)          |
| `color_array`       | `np.ndarray` | Decoded RGB image array (H×W×3)                   |
| `color`             | `np.ndarray` | Shortcut for `color_array`                        |
| **Depth Image**     |              |                                                   |
| `depth_array`       | `np.ndarray` | Depth image array (uint16, H×W)                   |
| `depth`             | `np.ndarray` | Shortcut for `depth_array`                        |
| `depth_width`       | `int`        | Depth image width                                 |
| `depth_height`      | `int`        | Depth image height                                |
| **Helpers**         |              |                                                   |
| `has_color_image`   | `bool`       | Check if color image exists                       |
| `has_depth_image`   | `bool`       | Check if depth image exists                       |

**Methods:**

| Method                                | Returns | Description                       |
| ------------------------------------- | ------- | --------------------------------- |
| `show_color(window_name)`             | `bool`  | Display color image with OpenCV   |
| `show_depth(window_name, colormap)`   | `bool`  | Display depth image with colormap |
| `show_images(show_color, show_depth)` | `tuple` | Display both images side by side  |

## Requirements

-   Python >= 3.8
-   numpy >= 1.20.0
-   pyzmq >= 22.0.0
-   protobuf >= 4.0.0
-   opencv-python >= 4.5.0
-   Pillow >= 8.0.0

## License

MIT License
