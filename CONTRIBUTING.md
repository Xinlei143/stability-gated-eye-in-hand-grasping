# Contributing

感谢改进本项目。机械臂运动修改必须区分“静态/plan-only验证”与“真机验证”。

## 开发流程

1. 从 `main` 创建短分支。
2. 只修改自研 `foam_grasp` 包；第三方驱动升级应单独提交并更新 `.repos`。
3. 运行 `scripts/validate_project.sh` 和 `scripts/check_github_ready.sh`。
4. 在Pull Request中说明是否做过plan-only、RViz、空载夹爪和真机运动验证。

## 不可提交的内容

- 手眼标定JSON、相机/机械臂序列号；
- 训练数据、rosbag、含人脸或敏感环境的图片；
- 模型权重、venv、colcon构建产物；
- Token、密码、SSH私钥和本地 `.env`。

## 真机安全

不要通过删除硬限位、碰撞检查、跟踪误差上限或发布者互斥检查来“修复”抓取。
修改默认速度、TCP、工作空间或夹爪行程时，必须附带风险说明和验证证据。
