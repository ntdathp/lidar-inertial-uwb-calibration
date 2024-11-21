# Fast and cost-effective UWB anchor position calibration using a portable SLAM system 

We propose a novel method to calibrate the location of ultrawideband (UWB) anchors using a rigidly attached simultaneous localization and mapping (SLAM)-UWB tag sensor system and solving the inverse problem of UWB positioning: based on known trajectories of the tags and UWB measurements relating the tags and the anchor, we estimate the position of the anchors. The proposed system has the potential to be a fast and cost-effective replacement for the traditional total station (TS) or full-room scanner solutions used to calibrate UWB anchors. To evaluate our system, we collect a dataset in a large warehouse in realistic conditions. We describe the design of our hardware-software calibration framework and its implementation, analyze our dataset, and explore how different robust loss functions affect the performance of our algorithm. 

Paper: [https://ieeexplore.ieee.org/document/10582837](https://ieeexplore.ieee.org/document/10582837)

## Build and run

In your ROS1 workspace:
```
cd src
git clone 
catkin build
```

Unfortunately, we cannot provide the dataset used in this work. The code can be used as reference for other implementations.



## Citation

```bibtex
@article{hamesse_fast_2024,
	title = {Fast and {Cost}-{Effective} {UWB} {Anchor} {Position} {Calibration} {Using} a {Portable} {SLAM} {System}},
	volume = {24},
	copyright = {https://ieeexplore.ieee.org/Xplorehelp/downloads/license-information/IEEE.html},
	issn = {1530-437X, 1558-1748, 2379-9153},
	url = {https://ieeexplore.ieee.org/document/10582837/},
	doi = {10.1109/JSEN.2024.3419261},
	number = {16},
	urldate = {2024-11-15},
	journal = {IEEE Sensors Journal},
	author = {Hamesse, Charles and Vleugels, Robbe and Vlaminck, Michiel and Luong, Hiep and Haelterman, Rob},
	month = aug,
	year = {2024},
	pages = {26496--26505},
}

```