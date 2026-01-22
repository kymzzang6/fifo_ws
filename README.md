# PPE 인식 패키지 실행 가이드

## 1. 워크스페이스 빌드

```
colcon build --symlink-install
```
## 2. 런치파일 실행

```
ros2 launch ppe_detector ppe.launch.py
````

# UI 실행 가이드

## 1. node.js 설치

https://nodejs.org/ko/download 참고

## 2. UI 실행

```
cd path/to/ui

npm install

chmod -R 755 node_modules/.bin

npm run dev
```