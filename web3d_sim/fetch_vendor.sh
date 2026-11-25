#!/usr/bin/env bash
# 下载前端第三方库 (three.js / roslib / STLLoader) 到 static/vendor/
# 仓库不携带第三方库文件, 首次运行前执行一次:
#   ./fetch_vendor.sh
set -e
cd "$(dirname "$0")"
mkdir -p static/vendor
BASE="https://cdn.jsdelivr.net/npm"
curl -fsSL -o static/vendor/three.module.js "$BASE/three@0.160.0/build/three.module.js"
curl -fsSL -o static/vendor/roslib.min.js "$BASE/roslib@1.3.0/build/roslib.min.js"
curl -fsSL -o static/vendor/STLLoader.js "$BASE/three@0.160.0/examples/jsm/loaders/STLLoader.js"
echo "vendor 库下载完成:"
ls -la static/vendor/
