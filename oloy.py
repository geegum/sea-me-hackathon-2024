import argparse
import os
import torch
from pathlib import Path
from models.experimental import attempt_load
from utils.datasets import LoadImages
from utils.general import check_img_size, non_max_suppression, scale_coords, xyxy2xywh
from utils.torch_utils import select_device

@torch.no_grad()
def run(weights='yolov5s.pt',
        source='/home/lee/yolov5/green',
        img_size=640,
        conf_thres=0.25,
        iou_thres=0.45,
        device='',
        save_txt=True,
        classes=[9],  # 9 is the class ID for traffic light in COCO dataset
        agnostic_nms=False,
        ):
    
    device = select_device(device)
    half = device.type != 'cpu'

    model = attempt_load(weights, map_location=device)
    stride = int(model.stride.max())
    imgsz = check_img_size(img_size, s=stride)
    if half:
        model.half()

    dataset = LoadImages(source, img_size=imgsz, stride=stride)
    
    if device.type != 'cpu':
        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))

    for img_data in dataset:
        path, img, im0s = img_data[0], img_data[1], img_data[2]

        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()
        img /= 255.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        pred = model(img, augment=False)[0]
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes=classes, agnostic=agnostic_nms)

        traffic_light_detected = False
        for i, det in enumerate(pred):
            p, im0 = Path(path), im0s.copy()

            txt_path = str(p.with_suffix('.txt'))
            if len(det):
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                for *xyxy, conf, cls in reversed(det):
                    if int(cls) == 9:  # 9 is the class ID for traffic light
                        traffic_light_detected = True
                        if save_txt:
                            # Normalize coordinates
                            xywh = xyxy2xywh(torch.tensor(xyxy).view(1, 4)).view(-1).tolist()
                            xywh = [xywh[0]/im0.shape[1], xywh[1]/im0.shape[0], 
                                    xywh[2]/im0.shape[1], xywh[3]/im0.shape[0]]  # normalize to 0-1
                            line = (int(cls), *xywh, conf)
                            with open(txt_path, 'a') as f:
                                f.write(('%g ' * len(line)).rstrip() % line + '\n')

        if not traffic_light_detected:
            os.remove(path)  # 신호등이 검출되지 않은 이미지 삭제
            if os.path.exists(txt_path):
                os.remove(txt_path)  # 관련 텍스트 파일도 삭제
        elif not save_txt:
            # 신호등이 검출되었지만 save_txt가 False인 경우, 빈 텍스트 파일 생성
            open(txt_path, 'w').close()

    print(f'Done. Results saved to {source}')

def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default='yolov5s.pt', help='model.pt path(s)')
    parser.add_argument('--source', type=str, default='/home/lee/yolov5/green', help='file/dir/URL/glob, 0 for webcam')
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--classes', nargs='+', type=int, default=[9], help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    opt = parser.parse_args()
    return opt

def main(opt):
    run(**vars(opt))

if __name__ == "__main__":
    opt = parse_opt()
    main(opt)
