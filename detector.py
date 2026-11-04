"""Person detector: YOLO11n (COCO) as a TensorRT FP16 engine on the Orin Nano.

  det = PersonDetector("models/yolo11n-fp16.engine")
  people = det(frame_bgr)   # [(x1, y1, x2, y2, score), ...] in frame pixels, best first

With the pose engine (yolo11n-pose-fp16.engine) each detection also carries
its 17 COCO keypoints as a (17, 3) array of x, y, confidence in frame pixels:
  [(x1, y1, x2, y2, score, keypoints), ...]

Build the engine once (models/ is git-ignored):
  /home/netra/ptz-node/.venv/bin/python -c "from ultralytics import YOLO; YOLO('yolo11n.pt').export(format='onnx', imgsz=640, opset=17, simplify=True)"
  /usr/src/tensorrt/bin/trtexec --onnx=yolo11n.onnx --saveEngine=yolo11n-fp16.engine --fp16
"""
import cv2
import numpy as np
import tensorrt as trt
from cuda.bindings import runtime as cudart

PERSON = 0  # COCO class id


def _check(res):
    err, *rest = res if isinstance(res, tuple) else (res,)
    if err != cudart.cudaError_t.cudaSuccess:
        raise RuntimeError(f"CUDA error {err}")
    return rest[0] if len(rest) == 1 else rest


class PersonDetector:
    def __init__(self, engine_path, conf=0.4, iou=0.5):
        self.conf, self.iou = conf, iou
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f:
            self.engine = trt.Runtime(logger).deserialize_cuda_engine(f.read())
        self.ctx = self.engine.create_execution_context()
        names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
        self.inp = next(n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT)
        self.out = next(n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT)
        self.in_shape = tuple(self.engine.get_tensor_shape(self.inp))    # (1, 3, 640, 640)
        self.out_shape = tuple(self.engine.get_tensor_shape(self.out))   # (1, 84, 8400)
        self.in_dtype = trt.nptype(self.engine.get_tensor_dtype(self.inp))
        self.out_dtype = trt.nptype(self.engine.get_tensor_dtype(self.out))
        self.size = self.in_shape[2]
        _check(cudart.cudaSetDevice(0))
        self.h_out = np.empty(self.out_shape, self.out_dtype)
        self.d_in = _check(cudart.cudaMalloc(int(np.prod(self.in_shape)) * np.dtype(self.in_dtype).itemsize))
        self.d_out = _check(cudart.cudaMalloc(self.h_out.nbytes))
        self.stream = _check(cudart.cudaStreamCreate())
        self.ctx.set_tensor_address(self.inp, int(self.d_in))
        self.ctx.set_tensor_address(self.out, int(self.d_out))

    def _letterbox(self, frame):
        h, w = frame.shape[:2]
        s = self.size / max(h, w)
        nw, nh = round(w * s), round(h * s)
        canvas = np.full((self.size, self.size, 3), 114, np.uint8)
        top, left = (self.size - nh) // 2, (self.size - nw) // 2
        canvas[top:top + nh, left:left + nw] = (frame if (nw, nh) == (w, h) else
                                                cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR))
        blob = cv2.dnn.blobFromImage(canvas, 1 / 255.0, swapRB=True)  # NCHW float32 RGB, ~2 ms faster than numpy
        return blob.astype(self.in_dtype, copy=False), s, left, top

    def __call__(self, frame):
        blob, s, left, top = self._letterbox(frame)
        _check(cudart.cudaMemcpyAsync(self.d_in, blob.ctypes.data, blob.nbytes,
                                      cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, self.stream))
        if not self.ctx.execute_async_v3(self.stream):
            raise RuntimeError("TensorRT execute_async_v3 failed")
        _check(cudart.cudaMemcpyAsync(self.h_out.ctypes.data, self.d_out, self.h_out.nbytes,
                                      cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost, self.stream))
        _check(cudart.cudaStreamSynchronize(self.stream))

        pred = self.h_out[0].astype(np.float32)  # (84, 8400): cx, cy, w, h, 80 class scores
        pose = pred.shape[0] == 56               # (56, 8400): cx, cy, w, h, person score, 17 x (x, y, conf)
        scores = pred[4] if pose else pred[4 + PERSON]
        keep = scores > self.conf
        if not keep.any():
            return []
        cx, cy, bw, bh = pred[0, keep], pred[1, keep], pred[2, keep], pred[3, keep]
        sc = scores[keep]
        kps = pred[5:, keep].T.reshape(-1, 17, 3) if pose else None
        x1 = (cx - bw / 2 - left) / s
        y1 = (cy - bh / 2 - top) / s
        boxes = np.stack([x1, y1, bw / s, bh / s], 1)
        idx = cv2.dnn.NMSBoxes(boxes.tolist(), sc.tolist(), self.conf, self.iou)
        h, w = frame.shape[:2]
        out = []
        for i in np.array(idx).reshape(-1):
            bx, by, bw_, bh_ = boxes[i]
            det = (max(0.0, bx), max(0.0, by), min(w, bx + bw_), min(h, by + bh_), float(sc[i]))
            if pose:
                k = kps[i].copy()
                k[:, 0] = (k[:, 0] - left) / s
                k[:, 1] = (k[:, 1] - top) / s
                det += (k,)
            out.append(det)
        return sorted(out, key=lambda b: -b[4])
