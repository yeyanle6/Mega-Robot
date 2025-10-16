# GitHub 上传指南

本文档指导如何将 Mega-Robot 项目上传到 GitHub 仓库 [yeyanle6/Mega-Robot](https://github.com/yeyanle6/Mega-Robot)。

---

## 📋 上传前检查清单

### 1. 确认文件结构

```bash
cd ~/Code/Demo5

# 检查必要文件是否存在
ls -la README.md LICENSE .gitignore
ls -la docs/
ls -la scripts/
ls -la src/t_robot_bringup/
ls -la src/t_robot_slam/
```

应该看到：
- ✅ `README.md` - 项目主文档
- ✅ `LICENSE` - Apache 2.0 许可证
- ✅ `.gitignore` - Git 忽略规则
- ✅ `docs/` - 文档目录
- ✅ `scripts/` - 测试脚本目录
- ✅ `src/` - ROS 2 包源代码

### 2. 检查临时文件

确保以下文件/目录已被 `.gitignore` 忽略：

```bash
# 这些目录应该存在但不会被提交
ls build/ install/ log/  # 编译产物

# 这些临时文件应该被忽略
ls frames*.pdf frames*.gv  # TF 树图
ls /tmp/rtabmap_export/    # 地图导出
```

如果有 27 个 PDF/GV 文件，可以先清理：

```bash
# 可选：清理临时文件（不影响功能）
rm -f frames*.pdf frames*.gv
```

### 3. 验证 .gitignore 配置

```bash
# 检查哪些文件会被 Git 跟踪
git status --ignored

# 应该显示 build/, install/, log/ 等目录被忽略
```

---

## 🚀 初次上传步骤

### 方式 1: 使用 HTTPS（推荐）

#### 1.1 初始化 Git 仓库

```bash
cd ~/Code/Demo5

# 初始化 Git
git init

# 添加远程仓库
git remote add origin https://github.com/yeyanle6/Mega-Robot.git

# 验证远程仓库
git remote -v
```

#### 1.2 添加文件并提交

```bash
# 添加所有文件（.gitignore 会自动过滤）
git add .

# 检查要提交的文件
git status

# 创建初始提交
git commit -m "feat: Initial commit - Mega-Robot v0.9

- Add complete ROS 2 robot platform with Megarover3 + Livox MID360
- Implement RTAB-Map 3D LiDAR SLAM with pointcloud preprocessing
- Add EKF sensor fusion (wheel odometry + IMU)
- Include Nav2 navigation stack integration
- Provide automated testing scripts and comprehensive documentation
- System validated on Jetson AGX Orin with JetPack 6.2

Milestone: M2 (RTAB-Map) 85% complete, awaiting field mapping test
"
```

#### 1.3 推送到 GitHub

```bash
# 检查远程分支
git branch -M main

# 首次推送（如果仓库已存在内容，使用 --force）
git push -u origin main

# 或强制推送（如果需要覆盖远程仓库）
# git push -u origin main --force
```

**注意**: 如果 GitHub 仓库已有内容，可能需要先拉取或强制推送。

---

### 方式 2: 使用 SSH（需配置 SSH 密钥）

#### 2.1 配置 SSH 密钥（首次）

```bash
# 生成 SSH 密钥（如果尚未生成）
ssh-keygen -t ed25519 -C "your_email@example.com"

# 启动 SSH 代理
eval "$(ssh-agent -s)"

# 添加私钥
ssh-add ~/.ssh/id_ed25519

# 复制公钥内容
cat ~/.ssh/id_ed25519.pub
```

在 GitHub 网页端添加 SSH 密钥：
1. Settings → SSH and GPG keys → New SSH key
2. 粘贴公钥内容并保存

#### 2.2 使用 SSH 推送

```bash
# 添加 SSH 远程仓库
git remote add origin git@github.com:yeyanle6/Mega-Robot.git

# 推送
git push -u origin main
```

---

## 🔄 后续更新流程

### 日常提交

```bash
# 查看修改
git status
git diff

# 添加修改的文件
git add src/t_robot_slam/params/rtabmap.yaml
git add docs/project_plan.md

# 提交（遵循 Conventional Commits 规范）
git commit -m "feat: Optimize RTAB-Map ICP parameters for indoor mapping

- Increase ICP voxel size to 0.08m for faster processing
- Enable libpointmatcher for better accuracy
- Update loop closure detection threshold

Test: Reduced mapping latency from 50ms to 35ms
"

# 推送到远程
git push
```

### Conventional Commits 格式

使用标准化提交信息：

```
<type>(<scope>): <subject>

<body>

<footer>
```

**类型 (type)**:
- `feat`: 新功能
- `fix`: Bug 修复
- `docs`: 文档更新
- `style`: 代码格式（不影响功能）
- `refactor`: 重构
- `perf`: 性能优化
- `test`: 测试相关
- `chore`: 构建/工具链更新

**示例**:
```bash
git commit -m "fix(bringup): Resolve EKF TF publishing conflict

- Set publish_tf: false in ekf.yaml
- Restore TF publishing in pub_odom node
- Update state_estimation.launch.py

Fixes #12
"
```

---

## 🏷️ 版本标签管理

### 创建版本标签

```bash
# 创建带注释的标签
git tag -a v0.9 -m "Version 0.9 - RTAB-Map Integration

Features:
- Complete Bringup system with EKF fusion
- RTAB-Map 3D LiDAR SLAM (static test passed)
- Pointcloud preprocessing pipeline
- Nav2 navigation (60% complete)

Status: Awaiting field mapping test
"

# 推送标签到远程
git push origin v0.9

# 或推送所有标签
git push --tags
```

### 版本命名规范

遵循 [Semantic Versioning](https://semver.org/):
- `v0.9` - 当前开发版本
- `v1.0` - 第一个稳定版本（完成移动建图测试）
- `v1.1` - 小版本更新（Nav2 优化）
- `v2.0` - 重大更新（如添加 RealSense）

---

## 🌿 分支管理

### 推荐分支策略

```bash
# 主分支（稳定版本）
main

# 开发分支
develop

# 功能分支
feature/rtabmap-optimization
feature/realsense-integration

# 修复分支
fix/ekf-tf-conflict
```

### 创建功能分支

```bash
# 从 main 创建新分支
git checkout -b feature/nav2-costmap-tuning

# 开发完成后合并
git checkout main
git merge feature/nav2-costmap-tuning

# 删除功能分支
git branch -d feature/nav2-costmap-tuning
```

---

## 📦 子模块管理（第三方包）

如果第三方包（如 `megarover3_ros2`, `livox_ros_driver2`）有独立的 Git 仓库，可以使用 Git Submodules。

### 添加子模块

```bash
# 移除现有目录
mv src/megarover3_ros2 /tmp/backup/

# 添加为子模块
git submodule add https://github.com/vstoneofficial/megarover_ros2.git src/megarover3_ros2

# 提交子模块配置
git add .gitmodules src/megarover3_ros2
git commit -m "chore: Add megarover3_ros2 as submodule"
```

### 克隆包含子模块的仓库

```bash
# 方式 1：递归克隆
git clone --recursive https://github.com/yeyanle6/Mega-Robot.git

# 方式 2：克隆后初始化子模块
git clone https://github.com/yeyanle6/Mega-Robot.git
cd Mega-Robot
git submodule update --init --recursive
```

---

## 📊 大文件处理（可选）

如果需要提交大型文件（如 rosbag, 地图数据库），建议使用 Git LFS。

### 安装 Git LFS

```bash
sudo apt install git-lfs
git lfs install
```

### 配置 LFS 跟踪

```bash
# 跟踪 rosbag 文件
git lfs track "*.bag"
git lfs track "*.db3"
git lfs track "*.db"

# 提交 .gitattributes
git add .gitattributes
git commit -m "chore: Configure Git LFS for large files"
```

**注意**: GitHub 免费版 LFS 存储限制为 1GB，建议将大文件存储在其他位置（如 Google Drive）并在文档中提供链接。

---

## ✅ 上传后验证

### 1. 检查 GitHub 网页

访问 https://github.com/yeyanle6/Mega-Robot 确认：
- ✅ README.md 正确显示
- ✅ 目录结构完整
- ✅ LICENSE 文件存在
- ✅ .gitignore 生效（build/install/log 未上传）

### 2. 测试克隆

在另一台机器或新目录测试：

```bash
cd /tmp
git clone https://github.com/yeyanle6/Mega-Robot.git test_clone
cd test_clone

# 检查文件完整性
ls -la src/
cat README.md
```

### 3. 验证文档链接

确保 README.md 中的链接都能正常访问：
- 文档内部链接（如 `docs/README_TEST.md`）
- 外部链接（ROS 2 官方文档等）

---

## 🛡️ 安全检查

### 避免提交敏感信息

```bash
# 检查是否包含敏感信息
git log --all --full-history --source -- '*password*'
git log --all --full-history --source -- '*secret*'
git log --all --full-history --source -- '*token*'

# 检查配置文件
grep -r "password" src/
grep -r "192.168.1" src/  # IP 地址（如果是内网可忽略）
```

### 已确认安全的配置

项目中的网络配置（如 MID360 IP `192.168.1.3`）属于内网地址，可以公开。

---

## 🔗 添加徽章（Badge）

在 GitHub 仓库设置完成后，可以添加状态徽章到 README.md：

### CI/CD 徽章（需配置 GitHub Actions）

```markdown
[![Build Status](https://github.com/yeyanle6/Mega-Robot/workflows/CI/badge.svg)](https://github.com/yeyanle6/Mega-Robot/actions)
```

### 其他徽章

- 许可证: `[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)`
- ROS 版本: `[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)`
- 平台: `[![Platform](https://img.shields.io/badge/Platform-Jetson_AGX_Orin-76B900.svg)](https://www.nvidia.com/jetson-orin/)`

---

## 🤝 协作开发

### 邀请协作者

1. GitHub 仓库页面 → Settings → Collaborators
2. 输入 GitHub 用户名并发送邀请

### Pull Request 流程

```bash
# 1. Fork 仓库（GitHub 网页端）

# 2. 克隆 Fork 仓库
git clone https://github.com/YOUR_USERNAME/Mega-Robot.git
cd Mega-Robot

# 3. 添加上游仓库
git remote add upstream https://github.com/yeyanle6/Mega-Robot.git

# 4. 创建功能分支
git checkout -b feature/my-improvement

# 5. 开发并提交
git commit -am "feat: Add awesome feature"

# 6. 推送到 Fork 仓库
git push origin feature/my-improvement

# 7. 在 GitHub 创建 Pull Request
```

---

## 📝 后续维护

### 定期同步

```bash
# 拉取最新更新
git pull origin main

# 查看提交历史
git log --oneline --graph --decorate --all

# 查看文件变更统计
git diff --stat
```

### 清理本地仓库

```bash
# 清理未跟踪的文件
git clean -fd

# 重置到远程状态（慎用）
git fetch origin
git reset --hard origin/main
```

---

## 🆘 常见问题

### 1. 推送失败（非快进更新）

```bash
# 拉取并合并远程更改
git pull --rebase origin main

# 或强制推送（慎用，会覆盖远程）
git push --force origin main
```

### 2. 文件太大无法推送

```bash
# 检查大文件
find . -type f -size +50M

# 移除大文件并使用 .gitignore
echo "large_file.bag" >> .gitignore
git rm --cached large_file.bag
git commit -m "chore: Remove large file from tracking"
```

### 3. 忘记添加 .gitignore

```bash
# 移除已跟踪的忽略文件
git rm -r --cached build/ install/ log/

# 提交更改
git commit -m "chore: Remove build artifacts from tracking"
git push
```

---

## 📞 联系方式

如有问题，请通过以下方式联系：
- GitHub Issues: https://github.com/yeyanle6/Mega-Robot/issues
- Email: （在 GitHub 个人资料中查看）

---

**下一步**: 完成上传后，更新 [项目规划](project_plan.md) 中的里程碑状态！
