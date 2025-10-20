[https://www.youtube.com/watch?v=2l__8MQJ4AA](https://www.youtube.com/watch?v=2l__8MQJ4AA)

# Perspective Cameras
- Rectangular box with hole on one side that forms image on opposite side
- modern cameras approximate perfect linear perspective

## Camera Obscura
- [Wikipedia definition](https://en.wikipedia.org/wiki/Camera_obscura)
- First ever camera obscura image by [Gemma Frisius](https://en.wikipedia.org/wiki/Gemma_Frisius) (1545)
![Gemma Frisius Camera Obscura](./images/gemma_frisius_camera_obscura.png)
  - To study solar eclipse
  - Tiny hole where object gets projected in the back of the room
- Projection inverted (upside down) and reversed (left to right) in next image
  - By [James Ayscough](https://en.wikipedia.org/wiki/James_Ayscough) (1755)
  ![James Ayscough Camera Obscura](./images/james_ayscough_camera_obscura.jpg)
- Long History
  - inspiration for palaeolithic cave paintings?
    - All speculation, no evidence for this
  - 4th century BC - described by Chinese philosopher Mozi
  - 4th centry BC - described by Greek philosopher Aristotle
  - 1000 - described in *The Book of Optics* by **Alhazen / Ibn al-Haytham**
  - 1604 - term "camera obscura" coined by **Kepler**
  - 17th century - used for painting by Vermeer
  - 19th century - photographic camera
  - 20th century - digital camera

## Pinhole Camera
![Vertical tree flip of pinhole camera](./images/tree_pinhole.png)

## Pinhole camera model
- "dark chamber with a pinhole in one side"

![black box with no rays just yet](./images/black_box_empty.png)
![black box with light rays](./images/black_box_light_rays.png)
![black box with image plane](./images/black_box_image_plane.png)
![black box point-to-point correspondence](./images/black_box_p_to_p_correspondence.png)
![black box correspondence origin](./images/black_box_correspondence_origin.png)
  - The center of the hole is $(0,0,0)$

![black box 2d origin on image plane](./images/black_box_2d_origin_on_image_plane.png)
![black box focal length](./images/black_box_focal_length.png)
![black box coordinate system](./images/black_box_z_coordinate_system.png)
  - z going left to right
  - y going up and down
  - x and X going in and out screen
  - Capital letters for 3D coordinate systems

![black box similar right triangles](./images/black_box_similar_right_triangles.png)
![black box central projection](./images/black_box_central_projection.png)
  - Lose one dimension
    - cannot go back
    - How the hell does one go back? Use AI for this?

![black box simplified model](./images/black_box_simplified_model.png)
  - Basically moving image from left to middle
  - Makes it simpler and - signs disappear from equations

___

- Where do be the lens?

## Perspective Camera

![iPhone 7 camera lenses](./images/iphone_7_camera_lenses.png)
  - Counter many distortions that can happen
  - Complicated because it makes pinhole camera correct

## Straight lines project as straight lines

![Straight lines project straight lines](./images/straight_lines_stay_straight.png)
  - The rulers are overlayed after the image is taken
  - The 3D straight lines in the real world stay straight in the 2D images as seen above

## Perspective Projection
![spheres don't project as circles](./images/spheres_dont_project_as_circles.png)
  - Off center spheres project as ellipses
  - Is a conic section
    - When sliced at image plane, get ellipse

## Spheres project as ellipses
![Spheres project as ellipses](./images/spheres_normal_and_wide_angle.png)
  - The wide angle camera in the left image projects the ping pong balls as ellipses

## Perspective camera
- "camera obscura" is very old concept but basis for modern cameras
- mathematics of *pinhole camera model* simple but nevertheless applicable for advanced digital cameras
- Do not like distortions from wide angle cameras
- We don't have to look at them since our computer vision systems do that for us
  - i.e. Augmented Reality apps
