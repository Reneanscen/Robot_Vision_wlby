# zme_fusion_map



# 整机障碍物地图方案

障碍物地图区别于SLAM地图、语义地图，专用于局部避障。



![Picture1](https://gitlab.weilaibuyuan.com/advanced_robot_algorithm/zme_fusion_map/-/blob/develop/doc/Picture1.png)



## 功能

### 1. 底盘移动避障

需要考虑机械臂状态，及其正在交互（如搬运箱子），计算出机器人整体的覆盖区，再参考地图避障

### 2. 机械臂移动避障

机械臂运动时候需实时参考避障地图，适当停障或者重新规划

### 3. 遥操作安全辅助

遥操作安全辅助与汽车类似，会主动触发降低损失。在遥操作过程中，可能会出现：图像延时、卡顿、画面模糊、有盲区，操作工会因疲劳、注意力分散、图像问题等导致操作失误，为了降低误操作的损失，需实时开启避障系统，停障。



## 问题

### 1. 多传感器视角不同

传感器的视角不同，会影响清除策略，需要在各自传感器的视角内清除自己的数据，然后再融合

### 2. 深度相机的盲区较大

需要保存下来盲区点，脱离传感器视角的障碍物点需要保存下来，并设置根据消失时长降低点概率的属性

### 3. 区分出静、动障碍物

使用AI分类出障碍物的语义，并标记静、动态障碍物

### 4. 去除车体、机械臂、交互物体的占据

机械臂交互时，会遮挡传感器，此时需去除机械臂的占据，若在移动过程中，需再调整车体的覆盖区

### 5. 地图格式

最终的目标是要全机器人避障，因此3D避障空间不可或缺。但是一阶段只有2D costmap，存在误清除情况，不能很好的保留视野外点，需考虑是否要做中间版本。

### 6. 点云稀疏

桌子、床等物品在点云数据呈现的是一整面的点，会非常多，是否可以对其稀疏，例如只考虑外轮廓，这样可以降低传输压力、计算压力。但是否会对机械臂的规划有影响？



## 实现

### 1. 输入

> **2D LIDAR**
>
> > sensor_msgs::LaserScan
> >
> > > FOV  $220^o$
>
> **TOF raw-data**
>
> > sensor_msgs::PointCloud
> >
> > >  FOV $120^o(H) * 56^o(V)$
>
> **Realsense D435i raw-data**
>
> > sensor_msgs::PointCloud
> >
> > > FOV  $86^o(H) * 57^o(V)$
>
> **GroundObjects**
>
> > sensor_msgs::PointCloud
> >
> > > 0: rope / 1: dogpoop / 2: cola
> > >
> > > 3: orange / 4: milk / 5: datawire
>
> **Semantic**
>
> > sensor_msgs::PointCloud
> >
> > > 0: sink / 1: toilet / 2: bedside_table
> > >
> > > 3: bed / 4: hand_washing_sink / 5: desktop
> > >
> > > 6: handle / 7: water_dispenser_face / 8: water_dispenser_side

### 2. 设计

```c++
//step0 初始化
void FusionMapNode::Init()
{
    //01 初始化一个3*3*1.2的地图空间
}
//step1 多线接收处理传感器数据
void FusionMapNode::sensorCallback()
{
    //01 初始化设置
    //02 降维 voxfilter setLeafSize(0.05, 0.05, 0.05)
}
//step2 融合处理主程序
void FusionMapNode::Process()
{
    //01 初始化判断，所有数据均接入才可开启
    //02 通过传感器时间、局部odom位姿、外参，把所有数据都转换到base_link下
    //03 将三类传感器分别对地图进行更新，raytracing / raycasting；并标明点的存在周期
    //04 语义类信息使用需再讨论，区分障碍物点的生命周期？
    //05 将地图直接发布
    //06 将地图压缩成2维发布
}

```



![esdf](https://gitlab.weilaibuyuan.com/advanced_robot_algorithm/zme_fusion_map/-/blob/develop/doc/esdf.jpg)



![esdf](https://gitlab.weilaibuyuan.com/advanced_robot_algorithm/zme_fusion_map/-/blob/develop/doc/esdf.png)



![raycasting](https://gitlab.weilaibuyuan.com/advanced_robot_algorithm/zme_fusion_map/-/blob/develop/doc/raycasting.png)

s--传感器中心位置

p--打到物体的位置

x--某个voxel的中心，sp连线会经过x



## 拓展

### BEV环视应用

使用BEV环视参与避障是否会提升避障效果，是否可以降低传感器成本





## Getting started

To make it easy for you to get started with GitLab, here's a list of recommended next steps.

Already a pro? Just edit this README.md and make it your own. Want to make it easy? [Use the template at the bottom](#editing-this-readme)!

## Add your files

- [ ] [Create](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#create-a-file) or [upload](https://docs.gitlab.com/ee/user/project/repository/web_editor.html#upload-a-file) files
- [ ] [Add files using the command line](https://docs.gitlab.com/ee/gitlab-basics/add-file.html#add-a-file-using-the-command-line) or push an existing Git repository with the following command:

```
cd existing_repo
git remote add origin http://gitlab.weilaibuyuan.com/advanced_robot_algorithm/zme_fusion_map.git
git branch -M main
git push -uf origin main
```

## Integrate with your tools

- [ ] [Set up project integrations](http://gitlab.weilaibuyuan.com/advanced_robot_algorithm/zme_fusion_map/-/settings/integrations)

## Collaborate with your team

- [ ] [Invite team members and collaborators](https://docs.gitlab.com/ee/user/project/members/)
- [ ] [Create a new merge request](https://docs.gitlab.com/ee/user/project/merge_requests/creating_merge_requests.html)
- [ ] [Automatically close issues from merge requests](https://docs.gitlab.com/ee/user/project/issues/managing_issues.html#closing-issues-automatically)
- [ ] [Enable merge request approvals](https://docs.gitlab.com/ee/user/project/merge_requests/approvals/)
- [ ] [Automatically merge when pipeline succeeds](https://docs.gitlab.com/ee/user/project/merge_requests/merge_when_pipeline_succeeds.html)

## Test and Deploy

Use the built-in continuous integration in GitLab.

- [ ] [Get started with GitLab CI/CD](https://docs.gitlab.com/ee/ci/quick_start/index.html)
- [ ] [Analyze your code for known vulnerabilities with Static Application Security Testing(SAST)](https://docs.gitlab.com/ee/user/application_security/sast/)
- [ ] [Deploy to Kubernetes, Amazon EC2, or Amazon ECS using Auto Deploy](https://docs.gitlab.com/ee/topics/autodevops/requirements.html)
- [ ] [Use pull-based deployments for improved Kubernetes management](https://docs.gitlab.com/ee/user/clusters/agent/)
- [ ] [Set up protected environments](https://docs.gitlab.com/ee/ci/environments/protected_environments.html)

***

# Editing this README

When you're ready to make this README your own, just edit this file and use the handy template below (or feel free to structure it however you want - this is just a starting point!). Thank you to [makeareadme.com](https://www.makeareadme.com/) for this template.

## Suggestions for a good README
Every project is different, so consider which of these sections apply to yours. The sections used in the template are suggestions for most open source projects. Also keep in mind that while a README can be too long and detailed, too long is better than too short. If you think your README is too long, consider utilizing another form of documentation rather than cutting out information.

## Name
Choose a self-explaining name for your project.

## Description
Let people know what your project can do specifically. Provide context and add a link to any reference visitors might be unfamiliar with. A list of Features or a Background subsection can also be added here. If there are alternatives to your project, this is a good place to list differentiating factors.

## Badges
On some READMEs, you may see small images that convey metadata, such as whether or not all the tests are passing for the project. You can use Shields to add some to your README. Many services also have instructions for adding a badge.

## Visuals
Depending on what you are making, it can be a good idea to include screenshots or even a video (you'll frequently see GIFs rather than actual videos). Tools like ttygif can help, but check out Asciinema for a more sophisticated method.

## Installation
Within a particular ecosystem, there may be a common way of installing things, such as using Yarn, NuGet, or Homebrew. However, consider the possibility that whoever is reading your README is a novice and would like more guidance. Listing specific steps helps remove ambiguity and gets people to using your project as quickly as possible. If it only runs in a specific context like a particular programming language version or operating system or has dependencies that have to be installed manually, also add a Requirements subsection.

## Usage
Use examples liberally, and show the expected output if you can. It's helpful to have inline the smallest example of usage that you can demonstrate, while providing links to more sophisticated examples if they are too long to reasonably include in the README.

## Support
Tell people where they can go to for help. It can be any combination of an issue tracker, a chat room, an email address, etc.

## Roadmap
If you have ideas for releases in the future, it is a good idea to list them in the README.

## Contributing
State if you are open to contributions and what your requirements are for accepting them.

For people who want to make changes to your project, it's helpful to have some documentation on how to get started. Perhaps there is a script that they should run or some environment variables that they need to set. Make these steps explicit. These instructions could also be useful to your future self.

You can also document commands to lint the code or run tests. These steps help to ensure high code quality and reduce the likelihood that the changes inadvertently break something. Having instructions for running tests is especially helpful if it requires external setup, such as starting a Selenium server for testing in a browser.

## Authors and acknowledgment
Show your appreciation to those who have contributed to the project.

## License
For open source projects, say how it is licensed.

## Project status
If you have run out of energy or time for your project, put a note at the top of the README saying that development has slowed down or stopped completely. Someone may choose to fork your project or volunteer to step in as a maintainer or owner, allowing your project to keep going. You can also make an explicit request for maintainers.
