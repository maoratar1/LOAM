# LiDAR Odometry and

# Mapping in Real Time

Explanation & Implementation

Maor Atar

# Little note about the report

• Paper explanations are written in black and the implementation parts which are written in purple.

• I supplied 2 .py files:

• LOAM_MAOR.py: main code.

• LOAM_MAOR_TEST.py: helper code. Plots/testing functions.

# 1 | Introduction


![](https://web-api.textin.com/ocr_image/external/bdba121130f28bb9.jpg)

## 1.1 | Introduction

• This paper deals with the problem of mapping and motion estimation using a 2-axis lidar.

• If the only motion of the lidar is to rotate a laser beam, registration of the point cloud is simple.

• But when the lidar itself is moving as in many applications of interest, accurate mapping requires:

1.Knowledge of the lidar pose during continuous laser ranging.

2.Correct the LiDAR point-cloud distortion.

• The method achieves both low-drift and low-computational complexity without the need for high accuracy ranging or inertial measurements. 


![](https://web-api.textin.com/ocr_image/external/597ed0b35dd6d82d.jpg)


• The key idea in obtaining this level of performance is the division of the typically complex problem of simultaneous localization and mapping (SLAM), which seeks to optimize a large number of variables simultaneously, by two algorithms.

• One algorithm performs odometry at a high frequency but low fidelity to estimate velocity of the lidar. 

• Another algorithm runs at a frequency of an order of magnitude lower for fine matching and registration of the point cloud.

• Specifically, both algorithms extract feature points located on sharp edges and planar surfaces and match the feature points to edge line segments and planar surface patches, respectively.


• To summarize:


|  | Frequency  | Fidelty  |
| --- | --- | --- |
| Odometry  | High 10 Hz  | Low  |
| Mapping  | Low 1 Hz  | High  |


• Both algorithms extract feature points :

• Edge point to edges lines

• Planar points to planar patches.


![](https://web-api.textin.com/ocr_image/external/125ed61b49883da6.jpg)


![](https://web-api.textin.com/ocr_image/external/68c4f7d5276f1d72.jpg)

## 1.2 | Lidar

# Hardware

<!-- Top  View Lidar -->
![](https://web-api.textin.com/ocr_image/external/b7095c19427a1d82.jpg)


![](https://web-api.textin.com/ocr_image/external/c32ac2f44c9262ef.jpg)


![](https://web-api.textin.com/ocr_image/external/b20934abfc4f2171.jpg)


![](https://web-api.textin.com/ocr_image/external/dd4ac0385b32c70e.jpg)

• Each line called a scan.

• Field of view of $180^{\circ }$  with $0.25^{\circ }$  resolution.

• 40 lines/sec scan rate.

• $\theta \in \left[-90^{\circ },90^{\circ }\right]$ 

• The laser scanner is attached to a motor rotating at $180^{\circ }$ /sec 

$$·φ\in \left[-90^{\circ },90^{\circ }\right]$$

• $(\theta ,φ,r)\mapsto (x,y,z)$  in Lidar CS.

<!-- $P(x,y,z)or(r,\theta ,φ)$ $z=r\cos \theta$ $x=r$ $y=r\sin \theta \sin \phi$ -->
![](https://web-api.textin.com/ocr_image/external/216b2677d1be1d42.jpg)

## 1.3 | Data

Implementation

·I used the **KITTI data** **set** (sequence 00).

·The background is the first point cloud.

**Vehicles**

## 1.3|Data

Implementation

<!-- Velodyne HDL-64E Laserscanner Point Gray Flea 2X Video Cameras KA EV842 -->
![](https://web-api.textin.com/ocr_image/external/0889e2f7d935ad20.jpg)


![](https://web-api.textin.com/ocr_image/external/32c6b934ba6eab75.jpg)


![](https://web-api.textin.com/ocr_image/external/2b3e7e7322b02e8b.jpg)

(a) (b)

Fig. 13. (a) Sensor configuration and vehicle used by the KITTI benchmark.The vehicle is mounted with a Velodyne lidar, stereo cameras, and a high accuracy GPS/INS for ground truth acquisition. Our method uses data from the Velodyne lidar only. (b) A sample lidar cloud (upper figure) and the corresponding visual image (lower figure) from an urban scene.

## 1.3 | Data

men

### First, I divided each point cloud into scans.

• I calculated the phi values given the x, y, z of the lidar.

• Velodyne HDL-64 ranges between [−𝟐𝟒. 𝟖°, 𝟐] with 64 scans.

• It can be seen that the separation to scans isn’t perfect.

• Next slide you could see the picture without shading.


![](https://web-api.textin.com/ocr_image/external/6c4bcfffa84bc951.jpg)


# 2 | Notations & Task Description


![](https://web-api.textin.com/ocr_image/external/271361aa887ba5ba.jpg)


![](https://web-api.textin.com/ocr_image/external/1d168089958b1c13.jpg)

## • Notations & Terms:

• Scan: Lidar completing a full scan along one axes.

• Sweep: Lidar completing a full scan across both axes.

•Kth Sweep: Occurs between timestamps $t_{k},t_{k+1}$ 

• $P_{k}$ : point-cloud at the kth sweep.

<!-- $k^{th}sweep=P_{k}$ -->
![](https://web-api.textin.com/ocr_image/external/4fc7b6d864a6a4a5.jpg)

•Each point in $P_{k}$ has a different time $\boldsymbol {t}\in \left[\boldsymbol {t}_{\boldsymbol {k}},\boldsymbol {t}_{\boldsymbol {k}+1}\right].$ 


![](https://web-api.textin.com/ocr_image/external/a56b66083cdbad77.jpg)

$LidarCS{L}$ and $X_{(k,i)}^{L}$ the i-th point in $P_{k}$ in Lidar CS.

$·WorldCS{W}$ and $X_{(k,i)}^{W}$ the i-th point in $P_{k}$ in World CS.

• $\boldsymbol {T}_{\boldsymbol {k}}^{\boldsymbol {L}}\in \mathbb {R}^{6}$ : ego-motion between $P_{k}$ and $P_{k+1}$ 

• Problem: $t_{k+1}$ 

• Given $\left\{P_{k}\right\}_{k=1}^{N}$ compute the ego-motion and build a map.

• Assumptions:

• Angular/Linear lidar’s velocities are smooth and continuous over time.


<!-- Implemen tation -->
![](https://web-api.textin.com/ocr_image/external/078b3d952c5cc053.jpg)

• A little note about the timestamps in my implementation: 

KITTI doesn’t supply a time stamp for each point in the Lidar scan but rather supplies some timestamp that you can use to calculate the timestamps for each point, but I couldn’t find a good and accurate way to do this and to avoid mistakes I omit this assumption so I’m not dealing with timestamps in my implementation at all. Which is not a mitigating assumption because the timestamp are used only when optimizing the transformation and instead of applying the original transformation, you should apply it consider the timestamp. So, it affects only on one line of code.

# 3| System Overview

## 3.2| Software & System overview

r

<!-- 4 z $t_{k+1}$ 근 0 $t_{k}$ y y k k $k+1$ $t_{k+1}$ 1Hz Undistorted $\mathcal {P}_{k}$ P Point Cloud $\mathcal {P}_{k}$ 1Hz Map Output Lidar Lidar Lidar Mapping Registration Odometry $T_{k}^{L}$ 1Hz Transform Update 10Hz Transform Real Update Transform Integration Bottle 10Hz Transform Output $\overline {\bar {Q}}_{k+1}\boldsymbol {T}_{k+1}^{L}$ $Q_{k}$ $\boldsymbol {T}_{k}^{W}$ -->
![](https://web-api.textin.com/ocr_image/external/949cde6de7c70499.jpg)

# 5 | Lidar Odometry

Overview

## The algorithm in a nutshell

• Iterative algorithm.

 Given $\bar {P}_{k},T_{k}^{L}:t_{k}\rightarrow t_{k+1}$ $\text {and}P_{k+1}$ we want to compute $T_{k+1}^{L}$ 


![](https://web-api.textin.com/ocr_image/external/60d388f9e3cb7af7.jpg)

<!-- $\overline {P_{k}}$ $P_{k+1}$ $t_{k+2}$ $T_{k+1}^{L}$ $T_{k}^{L}$ $t_{k+1}$ $tk+1$ $k+2$ -->
![](https://web-api.textin.com/ocr_image/external/c684eab354112a36.jpg)

## The algorithm in a nutshell

**Step** **1:** Find Edges/Planar Points in $P_{-}k$ 

$$P_{k+1}$$

·Edges Points

<!-- Pk $\overline {P_{k}}$ $T_{k}^{L}$ -->
![](https://web-api.textin.com/ocr_image/external/10e3172bcc1e4140.jpg)


![](https://web-api.textin.com/ocr_image/external/53023fc7d444eb89.jpg)

·Planar Pts

<!-- tk $tk+1$ 七 $k+2$ -->
![](https://web-api.textin.com/ocr_image/external/feac94f287fc642f.jpg)

$K+$ $x_{(k+1,i}$ The algorithm in a nutshell $\tilde {x}_{(k+1,i)}^{L}$  $\therefore$  $x^{2}(k+1,j)$ 

**Step** **2:** Reproject Feature Points Using Initial guess $\bar {T}_{k+1}^{L}$ $\tilde x(,z)$ 

Edges Points

<!-- Pk $\overline {\rho _{k}}$ Each point has its own time! $P_{k+1}$ $k+1$ ·Planar Pts 七 -->
![](https://web-api.textin.com/ocr_image/external/03690a791ad140b4.jpg)

tk


**Step** **3:** Find Edges-lines and Planar-patches correspondences

Pk+A

Edge

Point

<!-- Pk dε Pk $\overline {P_{k}}$ $\approx _{k+1}^{L}$ $P_{k+1}$ Edge line Pu+1 dH Pk Plahar t Patch tk $tk+1$ $k+2$ -->
![](https://web-api.textin.com/ocr_image/external/98cd3130386b1ce1.jpg)

Planar

Point


<!-- Pk $\frac {\pi _{k+1}^{L}}{}$ $P_{k+1}$ -->
![](https://web-api.textin.com/ocr_image/external/1972abc04bdb5ecd.jpg)

$$\text {Step}4:\text {argmin}_{T_{k+1}^{L}}f\left(X_{k+1}^{L},T_{k+1}^{L}\right)$$

$·X_{(k+1,i)}^{L}\in P_{k+1}$ : Edge Point.

$X_{(k+1,j)}^{L}\in P_{k+1}$ : Planar Point

<!-- Pk+1 Pk41 Planar L dH point Pk Plahar $T_{(k+1,i)}^{L}$ Patch -->
![](https://web-api.textin.com/ocr_image/external/8d2c39a1ba7247e0.jpg)

·Geometric relations via $T_{k+1}^{L}:$ 

$$f\left(X_{(k+1,i)}^{L},\right.\quad \left.T_{k+1}^{L}\right)=d_{ε}$$

$$f\left(X_{(k+1,j)}^{L},\right.\quad \left.T_{k+1}^{L}\right)=d_{H}$$

·Stacking all constraints :

$$\operatorname {argmin}_{T_{k+1}^{L}}\left\|f\left(T_{k+1}^{L}\right)\right\|$$

<!-- Pk+A Pk+1 dε Edge Pk Point $x_{i}$ Edgeline $T_{(k+1,i)}^{L}$ -->
![](https://web-api.textin.com/ocr_image/external/8cfd2bc61c704d81.jpg)

# 5 | Lidar Odometry

In Detail

## 5.1|Lidar Odometry

**Step** **1** | Feature Points Extraction

<!-- Pk $P_{k+1}$ ·EdgesPoints ·Planar Pts $T_{k}^{L}$ tk $tk+1$ $t_{k+2}$ -->
![](https://web-api.textin.com/ocr_image/external/08acc033b9e9ce50.jpg)

## 5.1 | Feature Point Extraction

• The laser scanner has a :

• 180° FOV with 0.25° resolution per scan.

• 40 Hz (40 lines/sec). 

• It rotates at 180°/sec, giving a 4.5° resolution perpendicular to the scan planes.

<!-- can  on  -->
![](https://web-api.textin.com/ocr_image/external/26d6e8b6845d4709.jpg)

• Points are located on a scan plane.

• Therefore, feature points are extracted using only information from individual scans.

• With co-planar geometric relationship.


·Selection feature points on **sharp** **edges** and **planar** **surface** patches.

<!-- 0 S -->
![](https://web-api.textin.com/ocr_image/external/61e7818c0a0ccdf5.jpg)

$\text {Let}i\in P_{k}$ and $S$  be the i-th consecutive points on the same scan.

·Smoothness evaluation:

**·Max** c value points are defined as **edge** points.

<!-- 乙 -->
![](https://web-api.textin.com/ocr_image/external/0a6d2db20e8ea8ce.jpg)

<!-- 2 -->
![](https://web-api.textin.com/ocr_image/external/5e36258c1f136baa.jpg)

**·Min** c value points are defined as **Planar** points.

·Up to some threshold.

·To evenly distribute feature points, each scan is divided into 4subregions, each providing up to 2 edge points and 4 planar points.

## 5.1 | Feature Point Extraction

## • Feature points selection criteria

• The number of selected edge points or planar points cannot exceed the maximum of the subregion, and

• None of its surrounding point is already selected, and

• It cannot be on a surface patch that is roughly parallel to the laser beam(a), or on boundary of an occluded region(b)

• I didn’t use the first filtering.


![](https://web-api.textin.com/ocr_image/external/26efbfb80506884e.jpg)


![](https://web-api.textin.com/ocr_image/external/7ee38457bf7a0112.jpg)


<!-- Implemen tation -->
![](https://web-api.textin.com/ocr_image/external/7fb50e426cb60ee0.jpg)

Implementation Key points:

• I divided each scan to 6.

• To calculate the c value, I took a window of size 10 around the point.

• I filter points that are not close to their neighbors.

• I determined thresholds for min and max c values (Which wasn’t so easy…)

• I filter almost all points that are lower than 20 cm from the ground.

<!-- 5.1 | Feature Point Extraction -->

<!-- Implementation -->

def find_feature_points_in_scan(scan):

$$\text {part_{size}=\text {len}(scan)//6}$$

part_size = max(part_size, 50) # Todo: consider divide it to 6 only if the scan size is higher than some number.

edge_points_total, planar_points_total, edge_points_indices_total, planar_points_indices_total = [], [], [], []

edges_c_vals_total, planar_c_vals_total = [],[]

for i in range(0, len(scan), part_size):

$$\text {start=}i$$

end = min(start + part_size, len(scan))

scan_part = scan[start:end]

edge_points, planar_points, edge_points_indices, planar_points_indices, edges_c_vals, planar_c_vals = find_feature_points_in_scan_part(scan_part)

edge_points_indices += start

planar_points_indices += start


Implementation

def find_feature_points_in_scan(scan):

part_size= len(scan)//6

$part_size=max(part_size,$ 50)# Todo: consider divide it to 6 only if the scan size is higher than some number.

edge_points_total, planar_points_total, edge_points_indices_total, planar_points_indices_total = [],[],[],[]

edges_c_vals_total,planar_c_vals_total =[],[]

for i in range(0, len(scan), part_size):

$$\text {start=}i$$

end = min(start + part_size,len(scan))

scan_part = scan[start:end]

edge_points, planar_points, edge_points_indices, planar_points_indices, edges_c_vals, planar_c_vals = find_feature_points_in_scan_part(scan_part)

edge_points_indices += start

planar_points_indices += start

def find_feature_points_in_scan_part(scan_part):

## 5.1| Feature Point Extraction Results

Edges,Planars


![](https://web-api.textin.com/ocr_image/external/3811318528013ff5.jpg)

<!-- 5.1 | Feature Point Extraction -->

<!-- Results -->

Good ones: Blue are the planar points and red are the edges.


![](https://web-api.textin.com/ocr_image/external/65c55e8be4060535.jpg)


![](https://web-api.textin.com/ocr_image/external/85eed5631d98cdc5.jpg)


![](https://web-api.textin.com/ocr_image/external/4625d57c23b79c6b.jpg)

<!-- 5.1 | Feature Point Extraction -->

<!-- Results -->

Bad ones: There's a bush in the left that made me troubles...


![](https://web-api.textin.com/ocr_image/external/9dfb40bd09fc9903.jpg)


<!-- Implemen tation -->
![](https://web-api.textin.com/ocr_image/external/d7dd3d5b25b4a40b.jpg)

Bad ones: Bush top view

• It can be seen that the c function approximation isn’t so good. The blue (planar) points has a very good c value but the they aren’t so planar.

• It influences the optimization process also.

• For a better implementation, one should think on a better way to define c or to reject those points. 

• I tried to calculate the points distribution in space with the covariance matrix, but it may be expensive.

• I tried also the reject points based on the diff vector (the one in the c function) values, but it didn’t lead to good results.

## 5.2|Lidar Odometry

**Step** **2** | Reprojecting Feature Points

**Pk** 

EdgesPoints

<!-- $\mathrm {T}_{\mathrm {k}+1}^{\mathrm {L}}$ -->
![](https://web-api.textin.com/ocr_image/external/da2b036aa67fdb3f.jpg)

PlanarPts

$$T_{h}^{L}$$

tk\quad t_{k+2}

## 5.2|Motion Estimation

·The lidar motion is modeled with **constant** **angular** **and** **linear** **velocities** during the sweep

$$\left[_{k+1},_{k+2}\right]$$

·Allowing **Linear Interpolation.**

<!-- OITuIod $\mathcal {P}_{k}$ $\overline {\mathcal {P}}_{k}$ $\mathcal {P}_{k+1}$ Time $t_{k}$ $t_{k+1}$ t -->
![](https://web-api.textin.com/ocr_image/external/a240878730a83945.jpg)

· $\boldsymbol {T}_{\boldsymbol {k}+\mathbf {1}}^{\boldsymbol {L}}\in \mathbb {R}^{6}$  6-DOF motion between $\left[t_{k+1},t_{k+2}\right]$ 

·Given $t_{i}$ its time stamp, and $\boldsymbol {T}_{(\boldsymbol {k}+\mathbf {1},i)}^{\boldsymbol {L}}$ be the motion $\left[t_{k+1},t_{i}\right]:$ 

$$\boldsymbol {T}_{(k+1,i)}^{L}=\frac {t_{i}-t_{k+1}}{t-t_{k+1}}\boldsymbol {T}_{k+1}^{L}\tag{4}$$

·It's a recursive algorithmm, we start with an initial guessof $T_{k+1}^{L}$ at with it and (4) we can project $E_{k+1}$ and $H_{k+1}$ to time $t_{k+1}$ ,denoted by $\hat {E}_{k+1}$ and $\widehat {H}_{k+1}$ respectively.


<!-- Implemen or each tation -->
![](https://web-api.textin.com/ocr_image/external/8302cd8581e0646f.jpg)

• As I said before, I didn’t find a good way to calculate the timestamp for each point in the point cloud (which is not given trivially by KITTI) but this relaxation doesn’t influence the algorithm almost at all. 

• If one asks to add it to the code, you just need to update the pose transformation when calculating the residuals in the optimization.

## 5.3| Lidar Odometry

**Step** **3** | Finding Feature Points Correspondences

Pk+A

X:EdgePoint

<!-- $P_{k}$ $\approx _{k+1}^{L}$ $P_{k+1}$ $\overline {P_{k}}$ tk x $k+1$ $t_{k+2}$ -->
![](https://web-api.textin.com/ocr_image/external/7d65031e096f6ecd.jpg)

<!-- dε Edgeline -->
![](https://web-api.textin.com/ocr_image/external/c642a518fe788e25.jpg)

<!-- $\overline {P}_{k+1}$ dH x Planar oint Plahar p patch -->
![](https://web-api.textin.com/ocr_image/external/93f539c9e19be50d.jpg)

## 5.2 | Finding Feature Point Correspondence

• Estimation of $T_{k+1}^{L}:$ $:t_{k+1}\rightarrow t_{k+2}$ 

• $\bar {P}_{k}:$ $P_{k}$ projected to timestamp $t_{k+1}$ 

• During the next sweep  is used together with the newly received point cloud, $P_{k+1},$ to estimate the motion of the lidar.

<!-- $\overline {\mathcal {P}}_{k}$ $\mathcal {P}_{k+1}$ $\mathcal {P}_{k}$ $t_{k}$ $t_{k+1}$ -->
![](https://web-api.textin.com/ocr_image/external/1f7dda6b7c4bd3ea.jpg)


• With $P_{k+1},$  we find edge points and planar points from the lidar cloud. Let $\boldsymbol {E}_{\boldsymbol {k}+1}\text {and}\boldsymbol {H}_{\boldsymbol {k}+1}\text {be}$ the set of edge points and planar points, respectively.

$$k+1,P_{k+1}$$

• With $\bar {P}_{k}$ we find edge lines/planar patches as the correspondences for points in $E_{k+1}\text {and}H_{k+1}.$ 

• The lidar odometry estimates recursively the motion $M_{\text {curr}}$ during the sweep and includes more points as $P_{k+1}$  increases. 

• At each iteration, $E_{k+1}\text {and}$ $H_{k+1}$  are reprojected to $t_{k+1}$  using $\boldsymbol {M}_{\text {curr}}·\text {Let}\widehat {\boldsymbol {E}}_{\boldsymbol {k}+\mathbf {1}}\text {and}$ $\widehat {\boldsymbol {H}}_{k+1}$ be the reprojected point sets. 

• For each point in $\widehat {E}_{k+1}\text {and}$  $\widehat {H}_{k+1}$ , we are going to find the closest neighbor point in $\bar {P}_{k}.$ 

• The lidar motion will be recover by minimizing the overall distances of the feature points.

<!-- $\overline {\mathcal {P}}_{k}$ $\mathcal {P}_{k+1}$ $\mathcal {P}_{k}$ $t_{k}$ $t_{k+1}$ -->
![](https://web-api.textin.com/ocr_image/external/313b26e4b4c0f9df.jpg)

$$c=\frac {1}{|\mathcal {S}|\cdot \left\|\boldsymbol {X}_{(k,i)}^{L}\right\|}\left\|\sum _{j\in \mathcal {S},j\neq i}\left(\boldsymbol {X}_{(k,i)}^{L}-\boldsymbol {X}_{(k,j)}^{L}\right)\right\|.$$


Finding point to edge line correspondence.

$$·i\in \hat {E}_{k+1}.$$

<!-- $P_{k}$ $\tilde {T}_{k+1}^{L}$ $P_{k+1}$ $T_{k}^{L}$ $tk+1$ $t_{k+2}$ -->
![](https://web-api.textin.com/ocr_image/external/ab97e45fb4572528.jpg)

$·j\in \widehat {P}_{k}$ be the closest neighbor of 𝑖.

• $e$ be the closest neighbor of $i$ in the two consecutive scans to the scan of j. 

• $(j,e)$  forms the line correspondence of $i$ .

• Verify that $j,e$ are edges using (1)


![](https://web-api.textin.com/ocr_image/external/e70ebae0f1cce7f6.jpg)

<!-- $i_{P_{k+1}}$ $\bar {P}_{k}$ -->
![](https://web-api.textin.com/ocr_image/external/ce0dd250a3fe60af.jpg)

## 5.3 | Finding Feature Point

## Correspondence

Finding **point** to **edge** line correspondence.

·Distance between i to its **edge-line** **correspondence** $(j,e)$ ):

$d_{\mathcal {E}}=\frac {\left|\left(\tilde {\boldsymbol {X}}_{(k+1,i)}^{L}-\overline {\boldsymbol {X}}_{(k,j)}^{L}\right)\times \left(\tilde {\boldsymbol {X}}_{(k+1,i)}^{L}-\overline {\boldsymbol {X}}_{(k,l)}^{L}\right)\right|}{\left|\overline {\boldsymbol {X}}_{(k,j)}^{L}-\left|\overline {\boldsymbol {X}}_{(k,l)}^{L}\right|\right.}$ ,

(2)


| $S=\vert a\times b\vert$ $=d_{\epsilon }\cdot c$ |
| --- |


<!-- i a b $d_{\epsilon }$ 子 C l -->
![](https://web-api.textin.com/ocr_image/external/14285d63e44cb1bf.jpg)

(1)**5.3 | Finding Feature Point** $c=\frac {1}{|\mathcal {S}|\cdot \left\|\boldsymbol {X}_{(k,i)}^{L}\right\|}\left\|\sum _{j\in \mathcal {S},j\neq i}\left(\boldsymbol {X}_{(k,i)}^{L}-\boldsymbol {X}_{(k,j)}^{L}\right)\right\|.$ 

# Correspondence

Finding **point** to **planar patch** correspondence.

<!-- $i_{P_{k+1}}$ m $\bar {P}_{k}$ Lidar -->
![](https://web-api.textin.com/ocr_image/external/254d55b36f722ce3.jpg)

·Let:

$$·i\in \widehat {H}_{k+1}$$

$·j\in \widehat {P}_{k}$ be the closest neighbor of $i$ i.

· $e,m$ be the closest neighbor of i. One in the same scan of j and the other in two consecutive scans to the scan of $j$ .

(b)

· $(j,l,m)$  forms the plane correspondence of $i$ .

·Verify $i,j,l,m$ are planar points

·Distance between $i$  to its planar $(j,l,m)$ :


| $d_{H}=\langle \hat {h},c\rangle$ $=$ $=\frac {\langle a\times b,c\rangle }{\vert \vert a\times b\vert \vert }$ |  |
| --- | --- |


<!-- i v dH a b 2 -->
![](https://web-api.textin.com/ocr_image/external/42798595f9b5fa44.jpg)

$$d_{\mathcal {H}}=\frac {\left|\begin{array}{c}\left(\tilde {\boldsymbol {X}}_{(k+1,i)}^{L}-\overline {\boldsymbol {X}}_{(k,j)}^{L}\right)\\ \left(\left(\overline {\boldsymbol {X}}_{(k,j)}^{L}-\overline {\boldsymbol {X}}_{(k,l)}^{L}\right)\times \left(\overline {\boldsymbol {X}}_{(k,j)}^{L}-\overline {\boldsymbol {X}}_{(k,m)}^{L}\right)\right)\end{array}\right|}{\left|\left(\overline {\boldsymbol {X}}_{(k,j)}^{L}-\overline {\boldsymbol {X}}_{(k,l)}^{L}\right)\times \left(\overline {\boldsymbol {X}}_{(k,j)}^{L}-\overline {\boldsymbol {X}}_{(k,m)}^{L}\right)\right|},\tag{3}$$

## 5.3|Finding Feature Point

1 usage Maor Atar

Implementation

### Correspondence def residuals_edge(Rt,edge_corr):

new_point = Rt[:,:3]@edge_corr.point +Rt[:,3]

a= new_point-edge_corr.e1

$$b=new_point-edge_corr.e2$$

c= edge_corr.e1-edge_corr.e2

r_line= np.linalg.norm(np.cross(a,b))/np.linalg.norm(c)

return r_line

1 usage Maor Atar

def residuals_plane(Rt,plane_corr):

new_point= Rt[:,:3] @plane_corr.point +Rt[:,3]

$$a=plane_corr.e1-plane_corr.e2$$

$$b=plane_corr.e1-plane_corr.e3$$

$$c=new_point-plane_corr.e1$$

$$ab_cross=np.cross(a,b)$$

abcross_dot_c= np.dot(ab_cross,c)

return np.linalg.norm(abcross_dot_c)/np.linalg.norm(ab_cross)

## 5.3|Finding Feature Point

Implementation

# Correspondence

def find_corresponding_edges_line_for_point(point_ind_in_scan, point_scan_ind, point, pcθ):

# j,L,m are notations from the paper

j_nn,j_idx_all_edges_lst, j_dist = search_nearest_neighbor(point, pc0.all_edges_pts)

j_nn, idx_all_edges_lst, $j_dist=j_nn[0],$ j_idx_all_edges_lst[e],j_dist[0] <u>#&nbsp;jidx&nbsp;it's&nbsp;the&nbsp;idx&nbsp;in&nbsp;th</u>e all edges pts lst

j_idx_in_pc = pc0.all_edges_pts_indices_in_pc[j_idx_all_edges_lst]

j_scan_id = pc0.point_ind_to_scan_ind(j_idx_in_pc)

j_ind_in_scan = j_idx_in_pc - pc0.scan_start[j_scan_id]

chosen_point,chosen_point_ind_in_scan, chosen_point_dist,chosen_scan_id = get_best_point_edge(j_scan_id,pc0,point)

if chosen_point is None:

return False,None

dist_thresh=1.

passed_dist_thresh = j_dist &lt; dist_thresh and chosen_point_dist &lt; dist_thresh

if not passed_dist_thresh:

return False,None

edge_corr = EdgeCorrespondence(point,

j_nn,j_dist,

chosen_point, chosen_point_dist,

point_ind_in_scan, j_ind_in_scan,chosen_point_ind_in_scan,

point_scan_ind, j_scan_id,chosen_scan_id)

return True,edge_corr

## 5.3|Finding Feature Point

Implementation

# Correspondence

def find_corresponding_planar_patch_for_point(point_ind_in_scan,point_scan_ind,point,pc0):

# j,l,m are notations fron the paper

j.nn,j_nn_idx_all_planar_lst,j_dist = search_nearest_neighbor(point,pco.all_planar_pts)

j_nn,j_nn_idx_all_planar_lst,j_dist =j_nn[e],j_nn_idx_all_planar_lst[0],j_dist[e]

j_idx_in_pc=pc0.all_planar_pts_indices_in_pc[j_nn_idx_all_planar_lst]

j_scan_id=pc0.point_ind_to_scan_ind(j_idx_in_pc)

j_ind_in_scan=j_idx_in_pc-pce.scan_start[j_scan_id]

two_nn,two_nn_idx_in_planar_points,two_dist=search_nearest_neighbor(point,pco.scans_obj_lst[j_scan_id].planar_points,num_nearest_neighbors=2) # best point in the sane scan 1_nn,l_nn_idx_in_planar_points, L_dist= two_nn[e], two_nn_idx_in_planar_points[0],two_dist[0]

L_scan_id=j_scan_id

l_ind_in_scan= pc0.scans_obj_lst[j_scan_id].planar_points_indices[1_nn_idx_in_planar_points]

if np.array_equal(L_nn,j_nn):

if len(two_nn)==2:

1_nn,1_nn_idx_in_planar_points,l_dist= two_nn[1],two_nn_idx_in_planar_points[1], two_dist[1]

L_ind_in_scan= pc0.scans_obj_lst[j_scan_id].planar_points_indices[1_nn_idx_in_planar_points]

else:

return False,None

chosen_point,chosen_point_ind_in_scan,chosen_point_dist,chosen_scan_id=get_best_point_planar(j_scan_id,pc0,point)

if chosen_point is None:

return False,None

dist_thresh=1.

passed_dist_thresh= j_dist &lt;dist_thresh and 1_dist&lt;dist_thresh and chosen_point_dist &lt; dist_thresh

if not passed_dist_thresh:

return False,None

planar_corr=PlanarCorrespondence(point,j_nn,1_nn,chosen_point,

j_dist,L_dist,chosen_point_dist,

point_ind_in_scan,j_ind_in_scan,l_ind_in_scan,chosen_point_ind_in_scan,

point_scan_ind,j_scan_id,L_scan_id,chosen_scan_id)

return True,planar_corr

# 5.3 | Finding Feature Point

<!-- Results -->
![](https://web-api.textin.com/ocr_image/external/7f8d9e1c9996de11.jpg)

# Correspondence

• Green lines represent the matching between Edges points in PC0 (darker gray) and PC1.


![](https://web-api.textin.com/ocr_image/external/d74902e0d926a847.jpg)


![](https://web-api.textin.com/ocr_image/external/04a153d9647e947a.jpg)

# 5.3 | Finding Feature Point

Correspondence Results

·And outliers... (from the annoying bush)

<!-- - - P Sg P . ...- -- -->
![](https://web-api.textin.com/ocr_image/external/a157b2690f27688e.jpg)

# 5.3 | Finding Feature Point

<!-- Results -->
![](https://web-api.textin.com/ocr_image/external/3b71b41940ab886d.jpg)

# Correspondence

• Green lines represent the matching between Planar points in PC0 (darker gray) and PC1.


![](https://web-api.textin.com/ocr_image/external/eb62e14b704f9413.jpg)


![](https://web-api.textin.com/ocr_image/external/ec1e3d2ebd29dae1.jpg)


![](https://web-api.textin.com/ocr_image/external/3f51ee4b732b49de.jpg)

## 5.3 | Finding Feature Point

Correspondence Results

·And the bush strikes again..


![](https://web-api.textin.com/ocr_image/external/3a028a1515129140.jpg)

## 5.4| Lidar Odometry

**Step** **4** | Solving

$$\operatorname {argmin}_{T_{k+1}^{L}}\left\|f\left(T_{k+1}^{L}\right)\right\|$$

·Combining (2) and (4)-(8), we can derive a **geometric** **relationship** between an edge point in $\mathrm {X}_{(k+1i}^{L}$ E $E_{k+1}$ and the corresponding edge line.

# 5.4| Motion Estimation

$$f_{\mathcal {E}}\left(\boldsymbol {X}_{(k+1,i)}^{L},\square {\boldsymbol {T}_{k+1}^{L}}\right)=d_{\mathcal {E}},i\in \mathcal {E}_{k+1}.\tag{9}$$

(2)

·Similarly for $H_{k+1}$ 

·Stacking all constraints together we get the equation:

$$\boldsymbol {f}\left(\boldsymbol {T}_{k+1}^{L}\right)=\boldsymbol {d},$$

·Solve as a **homogenous non-linear LS** (They use LM).

<!-- $\overline {P_{k}}+1$ PK+1 Edge L Pk dε Point i Edge line l $(k+1,i)$ -->
![](https://web-api.textin.com/ocr_image/external/a9e02b2de7144382.jpg)

(5)

## Remainders:

## Point line distance:

$$d_{\mathcal {E}}=\frac {\left|\left(\tilde {\boldsymbol {X}}_{(k+1,i)}^{L}-\overline {\boldsymbol {X}}_{(k,j)}^{L}\right)\times \left(\tilde {\boldsymbol {X}}_{(k+1,i)}^{L}-\overline {\boldsymbol {X}}_{(k,l)}^{L}\right)\right|}{\left|\overline {\boldsymbol {X}}_{(k,j)}^{L}-\overline {\boldsymbol {X}}_{(k,l)}^{L}\right|}$$

**Transformation interpolation:**

$$\left.\boldsymbol {T}_{(k+1,i)}^{L}\right]=\frac {t_{i}-t_{k+1}}{t-t_{k+1}}\boldsymbol {T}_{k+1}^{L}\tag{4}$$

**Reprojection edge point from** $t_{i}$ **to** $t_{k+1}:$ 

$$\boldsymbol {X}_{(k+1,i)}^{L}=\mathbf {R}\tilde {\boldsymbol {X}}_{(k+1,i)}^{L}+\boldsymbol {T}_{(k+1,i)}^{L}(1:3),$$

**Creating R from T angles:**

$$\theta =\left\|\boldsymbol {T}_{(k+1,i)}^{L}(4:6)\right\|,\tag{7}$$

$$\omega =\boldsymbol {T}_{(k+1,i)}^{L}(4:6)/\left\|\boldsymbol {T}_{(k+1,i)}^{L}(4:6)\right\|,\tag{8}$$

$$\mathbf {R}=e^{\hat {ω}\theta }=\mathbf {I}+\hat {ω}\sin \theta +\hat {ω}^{2}(1-\cos \theta )\tag{6}$$

# 5.4|Motion Estimation

Implementation

·I used the LM method using the following:

new*

def optimize(edge_corr,plane_corr):

pose_initial =np.array([0,0,0,0,0,0])

result = least_squares(residuals, pose_initial, args=(edge_corr,plane_corr),method="Lm")

return result.x

def residuals(pose, edges_corr,planes_corr):

all_residuals=[]

$$Rt=pose_to_mat(pose)$$

for edge_corr in edges_corr:

edge_r= residuals_edge(Rt, edge_corr)

all_residuals.append(edge_r)

for plane_corr in planes_corr:

plane_r= residuals_plane(Rt, plane_corr)

all_residuals.append(plane_r)

# print(all_residuals)

return np.array(all_residuals)

# 5.4 | Motion Estimation

<!-- ·Here you can see the PCO and PC1 before moving PO to PC1. Results -->
![](https://web-api.textin.com/ocr_image/external/48b83f251f3accc0.jpg)

# 5.4|Motion Estimation Results


![](https://web-api.textin.com/ocr_image/external/f82822b476f71d56.jpg)


![](https://web-api.textin.com/ocr_image/external/cd7b45683d3b79d2.jpg)

# 5.4 | Motion Estimation

<!-- Results -->
![](https://web-api.textin.com/ocr_image/external/124c3ac8f780840d.jpg)

• Here you can see the green PC which is PC0 transformed with the odometry result.

# 5.4 | Motion Estimation sul


![](https://web-api.textin.com/ocr_image/external/a87fe546e3f36a3e.jpg)

• Here you can see the green PC which is PC0 transformed with the odometry result.

• PC0 – cyan.

• PC1 – dark red.

• PC0_transformed – green.


![](https://web-api.textin.com/ocr_image/external/e75cfb164b5a528c.jpg)


![](https://web-api.textin.com/ocr_image/external/7927c1d592b4ae7a.jpg)

## Algorithm 1: Lidar Odometry

**input**: $\overline {\mathcal {P}}_{k},$ $\mathcal {P}_{k+1}$ $,\boldsymbol {T}_{k+1}^{L}$  from the last recursion

**2 output**: $\overline {\mathcal {P}}_{k+1}$ ,newly computed $\boldsymbol {T}_{k+1}^{L}$ 

3 **begin**

if at the beginning of a sweep **then**

5 $\vert \quad \boldsymbol {T}_{k+1}^{L}\leftarrow \mathbf {0}$ 

6 **end**

7 Detect edge points and planar points in $\mathcal {P}_{k+1},$  -put the points in

$\mathcal {E}_{k+1}$  and $\mathcal {H}_{k+1}$ ,respectively;

8 **for** a number of iterations **do**

9 **for** each edge point in $\mathcal {E}_{k+1}$ **do**

10

Find an edge line as the correspondence,then compute point to line distance based on $9$  and stack the equation to $11$ ;

11 **end**

12 **for** each planar point in $\mathcal {H}_{k+1}$ **do**

13 Find a planar patch as the correspondence, then compute

5.5|Algorithm point to plane distance based on $10$  and stack the

equation to $11$ ;

14 **end**

15 Compute a bisquare weight for each row of (11);

Update $\boldsymbol {T}_{k+1}^{L}$ for a nonlinear iteration based on (12);

16

17 if the nonlinear optimization converges **then**

18 Break;

19 **end**

20 **end**

21 if at the end of a sweep **then**

22 Reproject each point in $\mathcal {P}_{k+1}$  to $t_{k+2}$ and form $\overline {\mathcal {P}}_{k+1};$ 

23 Return $\boldsymbol {T}_{k+1}^{L}$ and $\overline {\mathcal {P}}_{k+1}$ 

24 end

25 else

26 Return $\boldsymbol {T}_{k+1}^{L};$ 

27 end

28end

Implementation

def lidar_odometry_between_2_pc_noTS(previous_undistorted_pc,Pk1,print_msg=False

if print_msg: print("Lidar O<u>&nbsp;</u><u>dometr</u>y Between-2 Points...")

scans= find_feature_points_in_point_cloud(Pk1)

Pk1.set_scans_obj_lst(scans)

corresponding_edges_lines=[]

corresponding_planar_patches =[]

# 5.5|Algorithm

if print_msg:print(". Searching for features points in point cloud...")

for scan_ind, scan in tqdm.tqdm(enumerate(scans),total=len(scans)):

corresponding_edges_lines+= find_corresponding_edges_lines(scan,previous_undistorted_pc)

corresponding_planar_patches += find_corresponding_planar_patches(scan,previous_undistorted_pc)

if print_msg:

print("Odometry: Num Edge constraints:", Len(corresponding_edges_lines))

print("Odometry:Num Planar constraints:"Len(corresponding_planar_patches))

result_1_to_0_pose = optimize(corresponding_edges_lines,corresponding_planar_patches)

if print_msg:

print("Odometry:1to0:", formatted_pose(result_1_to_θ_pose))

return pose_to_mat(result_1_to_0_pose),Pk1

6|Mapping

<!-- $\overline {\bar {Q}}_{k+1}\boldsymbol {T}_{k+1}^{L}$ $Q_{k}$ $\boldsymbol {T}_{k}^{W}$ -->
![](https://web-api.textin.com/ocr_image/external/9610a635cc87d877.jpg)

# Reminder

r

<!-- 4 z $t_{k+1}$ 근 0 y $t_{k}$ y k k $k+1$ $t_{k+1}$ 1Hz Undistorted $\mathcal {P}_{k}$ p Point Cloud $\mathcal {P}_{k}$ 1Hz Map Output Lidar Lidar Lidar Mapping Registration Odometry $T_{k}^{L}$ 1Hz Transform Update 10Hz Transform Real Update Transform Integration Bottle 10Hz Transform Output $\overline {\bar {Q}}_{k+1}\boldsymbol {T}_{k+1}^{L}$ $Q_{k}$ $\boldsymbol {T}_{k}^{W}$ -->
![](https://web-api.textin.com/ocr_image/external/5b3ca3b5c6d5ce76.jpg)

# 6 | Mapping

·Runs at a **lower frequency** then the odometry algorithm and it is called only **once** **per sweep.**

·Atthe endof sweep $k+1,$  we have in hand $\bar {P}_{k+1}$ in time $t_{k+2}$  and the motion $T_{k+1}^{L}:t_{k+1}\rightarrow t_{k+2}$ 

· Notations:

$.$ $Q_{k}$ Point cloud on the map accumulated until sweep k.

$T_{k}^{W}$ : Lidar pose on the map at the end of sweep k.

· $T_{k+1}^{W}=T_{k+1}^{L}\circ T_{k}^{W}$ 

$\bar {Q}_{k+1}$ : Point cloud from time $t_{k+2}$ projected to world CS using $T_{k+1}^{W}$ 

·Next the algorithm **matches** $\bar {Q}_{k+1}\text {wih}Q_{k}$ by optimizing $T_{k+1}^{W}$ 

<!-- $\bar {Q}_{k+1}\boldsymbol {T}_{k+1}^{L}$ $Q_{k}$ $\boldsymbol {T}_{k}^{W}$ -->
![](https://web-api.textin.com/ocr_image/external/06f0a57d1aed9304.jpg)

# 6| Mapping

**Matching** $\bar {Q}_{k+1}\text {with}Q_{k}$ 

·Feature points extracted in $\bar {Q}_{k+1}$ 

·10 times more of features are used.

<!-- QK @K+1 Edgeline Point -->
![](https://web-api.textin.com/ocr_image/external/7cc3c3f81a588e8c.jpg)

$·Q_{k}$ is stored in a 10m cubic area.

$\text {Store}\bar {Q}_{k+1}$ and $Q_{k}$ points that are on the same cubic.

·Let $S^{\prime }$ be a set of surrounding points from $Q_{k}$ 

·For an edge point, we only keep points on **edge** **lines** in $S^{\prime }$ .

·For a planar point, we only keep points on **planar patches**

·Now, search if $S^{\prime }$  is a line or a plane. GK+1Point


![](https://web-api.textin.com/ocr_image/external/6772eb67582b50d9.jpg)

QK PlaharPatch


![](https://web-api.textin.com/ocr_image/external/f07e2bd027166864.jpg)

<!-- $\bar {Q}_{k+1}$ 𝑆′ -->
![](https://web-api.textin.com/ocr_image/external/3644fd7f665ef295.jpg)

# 6 | Mapping

• Identifying if $S^{\prime }$  is a line or a plane using the covariance matrix of $S^{\prime }$  and it’s eigen-values and eigen vectors (Simply by doing SVD and looking at V and Σ).


![](https://web-api.textin.com/ocr_image/external/796087b10529ea3c.jpg)

• The position of $S^{\prime }$  is determined by its geometric center.

• Creating correspondences as we did before.

• Thus, we get optimized $T_{k+1}^{W}$ 

<!-- $\mathcal {P}_{k}$ $\mathcal {P}_{k}$ -->
![](https://web-api.textin.com/ocr_image/external/7846368003e02ee8.jpg)

<!-- 𝑆′ -->
![](https://web-api.textin.com/ocr_image/external/c750dc32cc6cbb87.jpg)

# 6|Mapping

Implementation

**·Edges Correspondences**

·For every feature point I search in a radius of $0.2m$  the corresponding edges points in the map.

·Here in **red, there are all the feature point of Pk,**in Cyan all the nearby points in the map and in **dark red are all the near feature points from the** **map.**

$$\therefore$$

. $----$ 

$v_{2}=v_{2}=v_{2}=0$ -

0

def·find_edge_corr(map, newPk, radius=0.2, ratio_threshold=5):

$0$ 051--

0

-

**6|Mapping**

Implementation

def find_edge_corr(map,newPk,radius=0.2,ratio_threshold=5):-

# Convert points to an Open3D point cloud

pcd=o3d.geometry.PointCLoud()

pcd.points = o3d.utility.Vector3dVector(map.points)

kdtree=o3d.geometry.KDTreeFLann(pcd)

corresponding_edges_lines=[]

for i,Pk_edge_point in enumerate(newPk.all_edges_pts):

[k,nearby_points_indices, _] = kdtree.search_radius_vector_3d(Pk_edge_point,radius)

nearby_edges_indices = (np.intersect1d(nearby_points_indices,map.edge_points_indices)).astype(int)nearby_edges = map.points[nearby_edges_indices, :]

if len(nearby_edges_indices) $>=2:$ 


![](https://web-api.textin.com/ocr_image/external/48b6065133974178.jpg)

<!-- $\therefore$ . $----$ $v_{2}=v_{2}=v_{2}=0$ - 0 $0$ 051 -- 0 - -->
![](https://web-api.textin.com/ocr_image/external/7a6baf3881b9b462.jpg)

$\text {centroid=np}$ .mean(nearby_edges,aaxis=0)

covariance_matrix=np.cov(nearby_edges- centroid, rowvar=False)

eigenvalues,eigenvectors = np.linalg.eigh(covariance_matrix)

if check_if_eig_values_represent_an_edge(eigenvalues, ratio_threshold=ratio_threshold):

edge_corr = MapEdgeCorr(Pk_edge_point,newPk.all_edges_pts_indices_in_pc[i],centroid,

centroid + eigenvectors[2], nearby_edges,nearby_edges_indices,

map.points[nearby_points_indices, :], nearby_points_indices)

#Assuming they are ordered

corresponding_edges_lines.append(edge_corr)

return corresponding_edges_lines

# 6|Mapping

Implementation

**·Planes Correspondences**

·For every feature point I search in a radius of 0.1m the corresponding edges points in the map.

--

-드

·Here in **Blue, there are all the feature point of Pk,**in Cyan all the nearby points in the map and in magenta are all the near feature points from the map.

- -

-드

--

--

--

..票：

-

)

Pg

P

*

<!-- def·find_plane_corr(map,newPk, radius=0.1, ratio_threshold=3) -->

**6|Mapping**

Implementation

def find_plane_corr(map, newPk,radius=0.1,ratio_threshold=3):

# Convert points to an Open3D point cloud

pcd= 03d.geometry.PointCloud()

pcd.points = o3d.utility.Vector3dVector(map.points)

kdtree= o3d.geometry.KDTreeFLann(pcd) ..

드 --


![](https://web-api.textin.com/ocr_image/external/887ddf2ac57a93cc.jpg)

corresponding_planar_patches=[]

-드

for i, newPk_plane_point in enumerate(newPk.all_planar_pts): - -

[k,nearby_points_indices,_]= kdtree.search_radius_vector_3d(newPk_plane_point,radius) -드

nearby_points = np.asarray(pcd.points)[nearby_points_indices,:] --

--

--

nearby_planar_indices = np.intersect1d(nearby_points_indices, map.planar_points_indices)

nearby_p<u>lanars</u> = map.points[nearby_planar_indices,:]

if len(nearby_planar_indices) $>=3:$ 票：

centroid= np.mean(nearby_points, axis=0)

covariance_matrix = np.cov(nearby_points -centroid, rowvar=False)

eigenvalues, eigenvectors=np.linalg.eigh(covariance_matrix)-

if check_if_eig_values_represent_a_plane(eigenvalues,ratio_threshold=ratio_threshold):

-pLane_corr=MapPLaneCorr(newPk_plane_point,newPk.all_planar_pts_indices_in_pc[i],

centroid, centroid + eigenvectors[2], centroid +eigenvectors[1], )

nearby_planars,nearby_planar_indices,

P

map.points[nearby_points_indices,:],nearby_points_indices)

# Assuming they are ordered

corresponding_planar_patches.append(plane_corr)

return corresponding_planar_patches

def mapping(cur2prev_matrices, P_ks, show_map=False, print_msg=0):

if print_msg $>=1:$ 

print("Start Mapping")

prev2map_mat = cur2prev_matrices[0] # Should be identity

world_map=Map(P_ks[θ].points,P_ks[0].all_edges_pts,P_ks[0].all_edges_pts_indices_in_pc, Lanarpts,

P_ks[0].all_planar_pts_indices_in_pc)

# 6|Mapping

Implementation

cam2map_matrices = [prev2map_mat]

for i in range(1, Len(cur2prev_matrices)): # index i represent the transform from i-1 to i

if print_msg:print("Working on Pc{}...".format(i))

cur_to_prev_mat = cur2prev_matrices[i]

initial_curr_to_map = compose_transformations(prev2map_mat,cur_to_prev_mat)

<u>P_k_in_mapinitial</u> = P_ks[i].apply_transformation(initial_curr_to_map)

<u>residual_curto_map&nbsp;=find_corrin_mapand_solve(world_map,Pkin_map_initial,printmsg)</u>

# Update Trans

cur2map_final = compose_transformations(residual_cur_to_map, initial_curr_to_map)

if print_msg&gt;=2:

print("Mapping: residual_cur_to_map:", formatted_pose(mat_to_pose(residual_cur_to_map)))

print("Mapping: cur2map_final:", formatted_pose(mat_to_pose(cur2map_final)))

# Update map

<u>Pk_in_</u>map = P_k_in_map_initial.apply_transformation(residual_cur_to_map)

world_map.add_points(P_k_in_map.points,P_k_in_map.all_edges_pts, P_k_in_map.all_edges_pts_indices_in_pc,

P_k_in_map.all_planar_pts, P_k_in_map.all_planar_pts_indices_in_pc)

# Color points from different pc in different colr to see diff

# Update for next iteration

prev2map_mat=cur2map_final

cam2map_matrices.append(cur2map_final)

if show_map:

world_map.show_map()

return cam2map_matrices

# 6|Mapping Results

The mapping improved the odometry result a bit.

Visualization in the next slide...


| Odometry:1to0: | [0.0010015417690943567,-0.0025269016111534055,0.005059975603542424,<br>0.6390697790297146,-0.031330652448839015,-0.015176447276427604] |
| --- | --- |
| Mapping:residual_cur_to_map: | [-0.00033937257803767795,-0.017601643715037956,-0.0018449851354327275<br>0.018462830599610847,0.025506878776466454,-0.038382210604279585]) |
| Mapping:cur2map_final: | [0.0006606384749475195,-0.02012921887727182,0.003196129046089532,<br>0.6575001088598266,-0.00569202884541152,-0.05348643281040568]) |


# 6|Mapping

PCO

·PC1 transformed with odometry

PC1 transformed with mapping.

<!-- Results -->
![](https://web-api.textin.com/ocr_image/external/73cc2e1565bcbc23.jpg)

# 6 | Mapping

<!-- Results -->
![](https://web-api.textin.com/ocr_image/external/d2ade3e6a55e4ed0.jpg)

• This is the trajectory of the first 8 frames.

• It has some noise in the x axes of about 1 cm (which isn’t so bad). 


![](https://web-api.textin.com/ocr_image/external/639e5894e00432b7.jpg)

# 6 | Mapping

<!-- Results -->
![](https://web-api.textin.com/ocr_image/external/2c3a0bab621c2b4c.jpg)

• Drift between the 7th frame and the 1st frame using the transformation I found

• PC7, PC7_transformed, PC0.

The pole can be a good reference to see the drift

<!-- T -->
![](https://web-api.textin.com/ocr_image/external/8939a01f6f4b262f.jpg)


![](https://web-api.textin.com/ocr_image/external/55f785423555dfc7.jpg)

# 7 | Experiments

## 7.1| Indoor & Outdoor Test

# Results


![](https://web-api.textin.com/ocr_image/external/9bd325664632a6f8.jpg)

(a)


![](https://web-api.textin.com/ocr_image/external/839afd6cdadc4dff.jpg)

(b)


![](https://web-api.textin.com/ocr_image/external/c85df8f4dfade6a6.jpg)

(c)


![](https://web-api.textin.com/ocr_image/external/ada521061d610f40.jpg)

(d)


![](https://web-api.textin.com/ocr_image/external/7c6fbb2a5a2cf2ba.jpg)


![](https://web-api.textin.com/ocr_image/external/052d85d97c331947.jpg)


![](https://web-api.textin.com/ocr_image/external/d95d00571214ac0d.jpg)

(e)

(g)

(h)


![](https://web-api.textin.com/ocr_image/external/13323a7f1f2d5e23.jpg)

(f)

Fig. 10. Maps generated in (a)-(b) a narrow and long corridor, (c)-(d)a large lobby, (e)-(f) a vegetated road, and (g)-(h) an orchard between two rows of trees. The lidar is placed on a cart in indoor tests, and mounted on a ground vehicle in outdoor tests. All tests use a speed of 0.5m/s.

## 7.1 | Indoor & Outdoor Test

## Map Evaluation

### • Compared to a ground-truth PC.

• The lidar is kept stationary.

• Placed at a few different places.

• The two PC are matched and compared using the point to plane ICP method

### • Matching errors are distances between a point cloud and corresponding planar patches in the other point-cloud.


![](https://web-api.textin.com/ocr_image/external/6663567e08c7d7ba.jpg)

### 7.1 | Indoor & Outdoor Test

## Accumulated Drift

·They choose a corridor for **indoor** experiments that contains a closed loop.

·This allows to start and finish at the same place.

·For **outdoor** experiments, they choose orchard environment.

·The ground vehicle that carries the lidar is equipped with a high accuracy GPS/INS for ground truth acquisition.

### TABLE I

RELATIVE ERRORS FOR MOTION ESTIMATION DRIFT.


|  | Environment  |  | Test 1  | Test 1  | Test 2  | Test 2  | Test 2  |
| --- | --- | --- | --- | --- | --- | --- | --- |
|  | Environment  |  | Distance  | Error  | Distance  | Error  |  |
|  | Corridor  |  | 58m  | 0.9% | 46m  | 1.1% |  |
|  | Orchard  |  | 52m  | 2.3% | 67m  | 2.8% |  |


### 7.1| Indoor & Outdoor Test

KITTI Dataset

<!-- Velodyne HDL-64E Laserscanner Point Gray Flea 2 Video Cameras KA EV 842 -->
![](https://web-api.textin.com/ocr_image/external/5c12bd4fcfbd3e06.jpg)

(a)


![](https://web-api.textin.com/ocr_image/external/5e099d0fd29f52a8.jpg)


![](https://web-api.textin.com/ocr_image/external/3bcd71618bc4a849.jpg)

(b)

Fig. 13. (a) Sensor configuration and vehicle used by the KITTI benchmark.The vehicle is mounted with a Velodyne lidar, stereo cameras, and a high accuracy GPS/INS for ground truth acquisition. Our method uses data from the Velodyne lidar only. (b) A sample lidar cloud (upper figure) and the corresponding visual image (lower figure) from an urban scene.

# 8| Conclusions

# 8 | Conclusions

• Motion estimation and mapping using point cloud from a rotating laser scanner can be difficult because the problem involves recovery of motion and correction of motion distortion in the lidar cloud. 

• The proposed method divides and solves the problem by two algorithms running in parallel: 

• the lidar odometry conduces coarse processing to estimate velocity at a higher frequency,

• while the lidar mapping performs fine processing to create maps at a lower frequency

• Cooperation of the two algorithms allows accurate motion estimation and mapping in real-time.

• Further, the method can take advantage of the lidar scan pattern and point cloud distribution. 

• Feature matching is made to ensure fast computation in the odometry algorithm,and to enforce accuracy in the mapping algorithm. 

• The method has been tested both indoor and outdoor as well as on the KITTI odometry benchmark

# 8 | Future Work

# Future Work

·Adding Loop closure mechanism.

# Future Work

<!-- Implemen tation -->
![](https://web-api.textin.com/ocr_image/external/7b5072726781ee49.jpg)

# • Run time isn’t so good.

• Use more matrix operations instead of for loop.

• Add more outlier's rejection policies.

• Use parallelization –

• Currently I calc the odometries first and than the mapping while it can be done in parallel.

• Currently I'm iterating over scans and searching for feature points which can be done in parallel.

• The searching of feature correspondences can be done I parallel also.

# • Use a probabilistic approach.

• Define a covariance matrix for each measurement and use mahalanobis distance to weight the constrains.

