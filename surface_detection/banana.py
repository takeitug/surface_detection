import cv2
import math
import numpy as np
import rclpy

class WidthScanner:
    def __init__(self, coef, mx, my):
        self._imx = int(mx)
        self._imy = int(my)
        self.delta = -1 / coef[0]
        self.b = my - mx * self.delta

    def getX(self, y):
        x = (y - self.b) / self.delta
        return x

    def getPoints(self, binimg):
        h,w = binimg.shape
        # 中央線より上をスキャン
        x1 = 0
        y1 = 0
        for y in range(self._imy, 0, -1):
            x = int(self.getX(y))
            if binimg[y,x] == 0:
                break
            else:
                x1 = x
                y1 = y

        # 中央線より下をスキャン
        x2 = 0
        y2 = 0
        for y in range(self._imy, h, 1):
            x = int(self.getX(y))
            if binimg[y,x] == 0:
                break
            else:
                x2 = x
                y2 = y
        dist = math.sqrt((x1-x2)**2+(y1-y2)**2)
        return (x1,y1), (x2, y2), dist
    

def main():
    img = cv2.imread('/home/isrlab/colcon_ws/src/manipulability/manipulability/banana.jpeg', cv2.IMREAD_COLOR)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, bw = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    bw = cv2.bitwise_not(bw)

    # モルフォロジー変換
    kernel5x5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, kernel5x5, iterations=2)

    target_lb_id = 1

    # ラベリング
    nLabels, labelImages, data, center = cv2.connectedComponentsWithStats(bw)
    tobjx = data[target_lb_id, 0]
    tobjy = data[target_lb_id, 1]
    tobjw = data[target_lb_id, 2]
    tobjh = data[target_lb_id, 3]
    h, w = bw.shape[:2]
    btarget = np.zeros((h, w), np.uint8)
    btarget[labelImages == target_lb_id] = 255

    img_mrg = cv2.merge((btarget,btarget,btarget)) 
    res = cv2.bitwise_and(img,cv2.merge((btarget,btarget,btarget)) )

    # 輪郭取得
    contours, _ = cv2.findContours(btarget, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for i, cnt in enumerate(contours):
        cv2.drawContours(res,[cnt],0,(0,0,255),1)

    # 回帰計算
    ypts, xpts = np.where(btarget == 255)
    rv = np.polyfit(xpts ,ypts,  3)
    expr = np.poly1d(rv)
    for x in range(tobjx, tobjx+tobjw):
        v = expr(x)
        res[int(v),x,:] = (255,255,0)   # 曲線描画

    # ターゲットオブジェクト認識領域描画
    cv2.rectangle(res,(tobjx,tobjy),(tobjx+tobjw,tobjy+tobjh),(0,255,255),1)
    # 幅を測る
    offsetx = 10
    incx = 16
    cary = np.zeros((incx,), np.float32)
    carx = np.zeros((incx,), np.float32)
    lbno = 1
    for sx in range(tobjx+incx, tobjx+tobjw-incx, incx):
        cary = np.zeros((incx,), np.uint8)
        for x in range(sx, sx+incx):
            vy = expr(x)
            carx[sx-x] = x
            cary[sx-x] = vy
        rv1 = np.polyfit(carx ,cary,1)
        expr1 = np.poly1d(rv1)

        mx = sx + incx // 2
        my = expr(mx)

        # 直行を求める
        ex = WidthScanner(rv1, mx, my)
        spt, ept, dist = ex.getPoints(bw)
        cv2.line(res, (int(spt[0]), int(spt[1])), (int(ept[0]), int(ept[1])), (255,0,0), 1, lineType=cv2.LINE_AA)

        cv2.putText(res, str(lbno), (int(spt[0]-6), int(spt[1])-8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,255), 1,cv2.LINE_AA)
        print('%2d.直径 %.2f' % (lbno, dist))
        lbno += 1

    cv2.imshow('res', res)
    cv2.imshow('bw', bw)
    cv2.imshow('btarget', btarget)
    cv2.waitKey(0)

if __name__ == '__main__':
    main()