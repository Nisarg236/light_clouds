This repo is still work in progress...

Some features to implement:
1) Ability to give multiple custom tags to points in a pointcloud, and access only the points with a specific tag (eg, corner, surface1, surface2, etc.)
   **example:**
    ```
    Point p1(1.0f, 1.0f, 1.0f);
    Point p2(1.0f, 1.1f, 1.4f);
    Point p3(1.0f, 1.2f, 1.3f);
    p3.addTag("corner");
    Point p4(1.0f, 1.3f, 1.2f);
    p4.addTag("corner");
    
    PointCloud cloud;
    cloud.addPoint(p1);
    cloud.addPoint(p2);
    cloud.addPoint(p3);
    cloud.addPoint(p4);
    
    std::set<std::string> filterTags = {"corner"};
    PointCloud filteredCloud = cloud.getSubsetByTags(filterTags);
    std::cout<< "the points with the tag corner are: ";
    filteredCloud.print();
    ```
3) No need to have a separate copy of original cloud and the filtered cloud just add a tag for example 'sor_filtered' to the filtered points in the existing cloud and get it easily when needed.
4) ROS1, ROS2 support.
5) Functions for popular feature extraction and matching algorithms.
6) Visualization?
