
# Light Weight Point Cloud Library

Welcome to **light_clouds**! This light weight library provides a powerful set of tools for working with 3D point clouds, specifically designed for robotics applications like 3D mapping, localisation, feature extraction and matching. With efficient data structures and direct impllementation of useful algorithms, this library allows you to process, filter, and analyze point cloud data for various robotics tasks.


> **Note**: This project is a work in progress, and many more features are being added continuously. Contributions are always welcome!

## Some Planned Features

- **Point and PointCloud Manipulation**:
   - Create and manage 3D points.
   - Perform common operations such as distance calculation, dot/cross products, and transformations.
   - Ability to add custom tags like "corner", "surface1", etc. to points in a point cloud and access/manipulate them based on tags.

- **Filtering**:
   - Apply filters for cleanup and noise removal.

- **Region of Interest (ROI)**:
   - Select an ROI from a point cloud  based on specified x, y, z limits.

- **File I/O**:
   - Load and save point clouds from and to PCD (Point Cloud Data) files, which is a standard format used in many point cloud applications.

- **Visualization Support** (Coming Soon):
   - A visualisation tool to visualise and edit and export the point cloud.

- **ROS Integration** (Planned):
   - Integration with ROS1 and ROS2 to make it easy use in applications.

---

## Getting Started

1. **Clone the repository**:

   ```bash
   git clone https://github.com/Nisarg236/light_clouds.git
   mkdir build && cd build
   cmake ..
   make
   ```

2. **Run the example**:
   ```
   ./light_clouds

   ```
---

3. **Run the test cases**:
   ```
   ctest --verbose

   ```
---
## Examples

### 1. **Creating and Manipulating Points**

```cpp
Point p1(1.0f, 2.0f, 3.0f);
Point p2(4.0f, 5.0f, 6.0f);
p1.print(1);
p2.print(0);

std::cout << "Distance between p1 and p2: " << p1.distanceTo(p2) << std::endl;

Point p3 = p1 + p2;
p3.print();

float dot_product = p1.dot(p2);
std::cout << "Dot product: " << dot_product << std::endl;

Point cross_product = p1.cross(p2);
cross_product.print();
```

### 2. **Working with PointClouds and Tags**

```cpp
Point p1(1.0f, 1.0f, 1.0f);
Point p2(1.0f, 1.1f, 1.4f);
Point p3(1.0f, 1.2f, 1.3f);
Point p4(1.0f, 1.3f, 1.2f);
p4.addTag("corner");
Point p5(10.0f, 1.4f, 1.1f);
p5.addTag("outlier");

PointCloud cloud;
cloud.addPoint(p1);
cloud.addPoint(p2);
cloud.addPoint(p3);
cloud.addPoint(p4);
cloud.addPoint(p5);

std::set<std::string> filterTags = {"corner"};
PointCloud filteredCloud = cloud.getSubsetByTags(filterTags);
filteredCloud.print();
```

### 3. **Apply Statistical Outlier Removal (SOR) Filter**

```cpp
// Print the points before filtering
std::cout << "Points in the PointCloud (before SOR filter):" << std::endl;
cloud.print();

// Apply SOR filter with k=2 neighbors and threshold=1.0f
Filters::applySOR(cloud, 2, 1.0f);

// Print the points after filtering
std::cout << "
Points in the PointCloud (after SOR filter):" << std::endl;
cloud.print();
```

### 4. **Region of Interest (ROI) Filtering**

```cpp
PointCloud::ROI roi = cloud.createROI(2.0, 8.0, 3.0, 10.0, 2.0, 7.0);

// Access the points inside the ROI
std::vector<Point> roiPoints = roi.getPoints();
for (const auto &point : roiPoints) {
    point.print(1);
}
```

### 5. **Loading and Saving PointClouds**

```cpp
PointCloud cloud;
cloud.loadFromPCD("input.pcd");
std::cout << "Cloud has: " << cloud.size() << " points." << std::endl;

cloud.dumpToPCD("output.pcd");
```

---

## Features Coming Soon

- **Advanced Feature Extraction**: Functions for detecting and matching key features in point clouds, such as corners, edges, and surfaces.
- **Visualization**: Tool to visualize point clouds, edit and export.
- **ROS Integration**: Support for ROS1 and ROS2 to make it easy to use in robotics applications.

---

## Contributing

Contributions are always welcome! Feel free to fork this project, submit pull requests, issues or add a star. Please follow these steps to contribute:

1. Fork the repository.
2. Create a new branch (`git checkout -b feature-branch`).
3. Make your changes and commit them (`git commit -m 'Add new feature'`).
4. Push to the branch (`git push origin feature-branch`).
5. Open a pull request.

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
