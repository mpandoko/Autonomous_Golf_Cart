#!/usr/bin/env python3
# limit the number of cpus used by high performance libraries
from distutils import text_file
import os
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

# sys.path.append(os.path.abspath(__file__+"/../"))
# sys.path.append(os.path.abspath(__file__+"/../yolov5/"))

import sys
import argparse
import platform
import shutil
import time
from pathlib import Path
import cv2
import torch
import torch.backends.cudnn as cudnn
import pyrealsense2 as rs
# PKG = 'numpy_tutorial'
# import roslib; roslib.load_manifest(PKG)
import rospy

from persepsi.yolov5.models.experimental import attempt_load
from persepsi.yolov5.utils.downloads import attempt_download
from persepsi.yolov5.models.common import DetectMultiBackend
from persepsi.utils2.datasets import LoadRealSense2, LoadImages, LoadStreams
from persepsi.yolov5.utils.general import (LOGGER, check_img_size, non_max_suppression, scale_coords, 
                                  check_imshow, xyxy2xywh, increment_path)
from persepsi.yolov5.utils.torch_utils import select_device, time_sync
from persepsi.yolov5.utils.plots import Annotator, colors
from persepsi.deep_sort.utils.parser import get_config
from persepsi.deep_sort.deep_sort import DeepSort
from persepsi.utils2.object_coordinate import *
from persepsi.msg import obj_points
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

### ROS

rospy.init_node('camera_node')

print('Creating ROS Publisher...')

# Get parameters
freq = rospy.get_param('~freq', 30.) # Hz

# Create publisher
# pub = rospy.Publisher('/object_points', numpy_msg(Floats), queue_size = 1)
pub = rospy.Publisher('/object_points', obj_points, queue_size = 1)
rate = rospy.Rate(freq) # Hz

msg = obj_points()
msg.header.frame_id = 'object_detection'
msg.header.seq = 0
msg.header.stamp = rospy.Time.now()
last_time = msg.header.stamp.to_sec() - 1./freq


FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # yolov5 deepsort root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative




def detect(opt):
    out, source, yolo_model, deep_sort_model, show_vid, save_vid, save_txt, imgsz, evaluate, half, project, name, exist_ok= \
        opt.output, opt.source, opt.yolo_model, opt.deep_sort_model, opt.show_vid, opt.save_vid, \
        opt.save_txt, opt.imgsz, opt.evaluate, opt.half, opt.project, opt.name, opt.exist_ok
    webcam = source == '0' or source.startswith(
        'rtsp') or source.startswith('http') or source.endswith('.txt')

    device = select_device(opt.device)
    # initialize deepsort
    cfg = get_config()
    cfg.merge_from_file(opt.config_deepsort)
    deepsort = DeepSort(deep_sort_model,
                        device,
                        max_dist=cfg.DEEPSORT.MAX_DIST,
                        max_iou_distance=cfg.DEEPSORT.MAX_IOU_DISTANCE,
                        max_age=cfg.DEEPSORT.MAX_AGE, n_init=cfg.DEEPSORT.N_INIT, nn_budget=cfg.DEEPSORT.NN_BUDGET,
                        )

    # Initialize
    half &= device.type != 'cpu'  # half precision only supported on CUDA

    # The MOT16 evaluation runs multiple inference streams in parallel, each one writing to
    # its own .txt file. Hence, in that case, the output folder is not restored
    if not evaluate:
        if os.path.exists(out):
            pass
            shutil.rmtree(out)  # delete output folder
        os.makedirs(out)  # make new output folder

    # Directories
    exp_name = yolo_model.split(".")[0] if type(yolo_model) is str else "ensemble"
    exp_name = exp_name + "_" + deep_sort_model
    # save_dir = increment_path(Path(project) / exp_name, exist_ok=exist_ok)  # increment run if project name exists
    # save_dir.mkdir(parents=True, exist_ok=True)  # make dir

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(yolo_model, device=device, dnn=opt.dnn)
    stride, names, pt, jit, _ = model.stride, model.names, model.pt, model.jit, model.onnx
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Half
    half &= pt and device.type != 'cpu'  # half precision only supported by PyTorch on CUDA
    if pt:
        model.model.half() if half else model.model.float()

    # Set Dataloader
    vid_path, vid_writer = None, None
    # Check if environment supports image displays
    if show_vid:
        show_vid = check_imshow()

    # Dataloader
    if webcam:
        show_vid = check_imshow()
        cudnn.benchmark = True  # set True to speed up constant image size inference
        #dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt and not jit)
        dataset = LoadRealSense2(width = 848, height = 480, fps = 30, img_size = imgsz) #best resolution for accuracy
        bs = len(dataset)  # batch_size
    else:
        dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt and not jit)
        bs = 1  # batch_size
    vid_path, vid_writer = [None] * bs, [None] * bs

    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names

    # extract what is in between the last '/' and last '.'
    #txt_file_name = source.split('/')[-1].split('.')[0]
    txt_file_name = 'r'
    txt_path = os.path.abspath(__file__+"/../inference/output/" + txt_file_name + '.txt')
    time_start = time_sync()

    if pt and device.type != 'cpu':
        model(torch.zeros(1, 3, *imgsz).to(device).type_as(next(model.model.parameters())))  # warmup
    dt, seen = [0.0, 0.0, 0.0, 0.0], 0
    for frame_idx, (path, depth, distance, depth_scale, img, im0s, color_intrin, aligned_df, verts, vid_cap, s) in enumerate(dataset):
        if rospy.is_shutdown():
            break
        t1 = time_sync()
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        t2 = time_sync()
        dt[0] += t2 - t1

        # Inference
        visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if opt.visualize else False
        pred = model(img, augment=opt.augment, visualize=visualize)
        t3 = time_sync()
        dt[1] += t3 - t2

        # Apply NMS
        pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, opt.classes, opt.agnostic_nms, max_det=opt.max_det)
        dt[2] += time_sync() - t3

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            seen += 1
            if webcam:  # batch_size >= 1
                p, im0, _ = path[i], im0s[i].copy(), dataset.count
                s += f'{i}: '
            else:
                p, im0, _ = path, im0s.copy(), getattr(dataset, 'frame', 0)

            p = Path(p)  # to Path
            # save_path = str(save_dir / p.name)  # im.jpg, vid.mp4, ...
            s += '%gx%g ' % img.shape[2:]  # print string

            annotator = Annotator(im0, line_width=2, pil=not ascii)

            if det is not None and len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(
                    img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                xywhs = xyxy2xywh(det[:, 0:4])
                confs = det[:, 4]
                clss = det[:, 5]
                z = calcdepth(xywhs, aligned_df)      
                xywhzs = torch.cat((xywhs, z), 1)
                xywhzs = xywhzs[xywhzs[:, 4] > 0.0] #delete all z = 0 readings

                # pass detections to deepsort
                t4 = time_sync()
                if (all(x.item() > 0 for x in z)): #prevent zero data readings, mp thinks this line is unecessary but just for redundancy
                    outputs = deepsort.update(xywhzs.cpu(), confs.cpu(), clss.cpu(), im0, color_intrin)
                    #outputs = deepsort.update(xywhs.cpu(), confs.cpu(), clss.cpu(), im0)
                    t5 = time_sync()
                    dt[3] += t5 - t4
                    total_time = t5-time_start

                    # draw boxes for visualization
                    if len(outputs) > 0:
                        x = np.array([])
                        y = np.array([])
                        z = np.array([])
                        obj_len = np.array([])
                        zc = np.array([])
                        xc = np.array([])
                        vxc = np.array([])
                        vzc = np.array([])
                        # obj = np.ones((1, 3))*1.0e5
                        num_obj = 0
                        for j, (output, conf) in enumerate(zip(outputs, confs)):
                            x_m = output[0]
                            y_m = output[1]
                            z_m = output[4] 
                            xy = rs.rs2_project_point_to_pixel(color_intrin, [x_m, y_m, z_m])
                            width = 848
                            height = 480
                            vx = output[5] / (t5-t1)
                            vz = output[9] / (t5-t1)
                            w = output[2]
                            h = output[3]
                            x1 = max(int(xy[0] - w / 2), 0) + 1
                            x2 = min(int(xy[0] + w / 2), width - 1)
                            y1 = max(int(xy[1] - h / 2), 0) + 1
                            y2 = min(int(xy[1] + h / 2), height - 1)
                            object_points = verts[y1:y2, x1:x2].reshape(-1, 3)
                            bboxes = [int(x1), int(y1), int(x2), int(y2)]
                            id = int(output[10])
                            cls =int(output[11])

                            # Get z points
                            if not object_points.size:
                                break
                            zs = object_points[:, 2]
                            
                            # Get z percentile 40 (where most likely the object blob is, 
                            # think of it like medians but more likely to pick the foreground object instead of the background)
                            z_ = np.percentile(zs, 40)

                            # Delete object points if less or greater than threshold
                            ## Threshold: z_ - 0.5 | z_ + 0.5
                            object_points = np.delete(object_points, np.where(
                                                    (zs < z_ - 0.5) | (zs > z_ + 0.5)), 0)
                            x0 = object_points[::50, 0]
                            y0 = object_points[::50, 1]
                            z0 = object_points[::50, 2]
                            obj_len0 = len(x0)
                            x = np.append(x, x0) #point cloud
                            y = np.append(y, y0) #point cloud
                            z = np.append(z, z0) #point cloud
                            obj_len = np.append(obj_len, obj_len0)
                            zc = np.append(zc, z_m) #z center
                            xc = np.append(xc, x_m) #x center
                            vxc =np.append(vxc, vx)
                            vzc =np.append(vzc, vz)

                            c = int(cls)  # integer class
                            label = f'{id} {names[c]} {conf:.2f}'
                            annotator.box_label(bboxes, label, color=colors(c, True))

                            if save_txt:
                                # Write for visualization Live_Plot
                                with open(txt_path, 'a') as f:
                                    f.write(('%g ' * 10 + '\n') % (frame_idx + 1, id, x_m,
                                                            z_m, vx, vz, total_time, -1, -1, -1))
                                                                                  
                        # Publish message
                        msg.header.seq += 1
                        msg.obj_len = obj_len
                        
                        msg.obj_x = x
                        msg.obj_y = y
                        msg.obj_z = z
                        msg.xc = xc
                        msg.zc = zc
                        msg.vxc = vxc
                        msg.vzc = vzc

                        print('x,z = (',xc,zc,')')
                        print('vx,vz = (',vxc,vzc,')')
                        
                        pub.publish(msg)
                    
                    else:
                        # Publish message
                        msg.header.seq += 1
                        msg.obj_len = []
                        
                        msg.obj_x = []
                        msg.obj_y = []
                        msg.obj_z = []
                        msg.xc = []
                        msg.zc = []
                        msg.vxc = []
                        msg.vzc = []
                        
                        pub.publish(msg)


                    LOGGER.info(f'{s}Done. YOLO:({t3 - t2:.3f}s), DeepSort:({t5 - t4:.3f}s)')

            else:
                deepsort.increment_ages()
                LOGGER.info('No detections')

            # Stream results
            im0 = annotator.result()
            if show_vid:
                cv2.imshow(str(p), im0)
                if cv2.waitKey(1) == ord('q'):  # q to quit
                    raise StopIteration

            # Save results (image with detections)
            if save_vid:
                if vid_path != save_path:  # new video
                    vid_path = save_path
                    if isinstance(vid_writer, cv2.VideoWriter):
                        vid_writer.release()  # release previous video writer
                    if vid_cap:  # video
                        fps = vid_cap.get(cv2.CAP_PROP_FPS)
                        w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    else:  # stream
                        fps, w, h = 30, im0.shape[1], im0.shape[0]

                    vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                vid_writer.write(im0)

    # Print results
    t = tuple(x / seen * 1E3 for x in dt)  # speeds per image
    LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS, %.1fms deep sort update \
        per image at shape {(1, 3, *imgsz)}' % t)
    if save_txt or save_vid:
        print('Results saved to %s' % save_path)
        if platform == 'darwin':  # MacOS
            os.system('open ' + save_path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--yolo_model', nargs='+', type=str, default='yolov5m.pt', help='model.pt path(s)')
    parser.add_argument('--deep_sort_model', type=str, default='osnet_x0_25')
    parser.add_argument('--source', type=str, default='0', help='source')  # file/folder, 0 for webcam
    parser.add_argument('--output', type=str, default='inference/output', help='output folder')  # output folder
    parser.add_argument('--imgsz', '--img', '--img-size', nargs='+', type=int, default=[640], help='inference size h,w')
    parser.add_argument('--conf-thres', type=float, default=0.5, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.5, help='IOU threshold for NMS')
    parser.add_argument('--fourcc', type=str, default='mp4v', help='output video codec (verify ffmpeg support)')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--show-vid', default='true', help='display tracking video results')
    parser.add_argument('--save-vid', action='store_true', help='save video tracking results')
    parser.add_argument('--save-txt', default='true', help='save MOT compliant results to *.txt')
    # class 0 is person, 1 is bycicle, 2 is car... 79 is oven
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 16 17')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--evaluate', action='store_true', help='augmented inference')
    parser.add_argument("--config_deepsort", type=str, default="deep_sort/configs/deep_sort.yaml")
    parser.add_argument("--half", action="store_true", help="use FP16 half-precision inference")
    parser.add_argument('--visualize', action='store_true', help='visualize features')
    parser.add_argument('--max-det', type=int, default=1000, help='maximum detection per image')
    parser.add_argument('--dnn', action='store_true', help='use OpenCV DNN for ONNX inference')
    parser.add_argument('--project', default=ROOT / 'runs/track', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand

    with torch.no_grad():
        detect(opt)
