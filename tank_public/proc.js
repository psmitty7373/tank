let video = document.getElementById('remoteVideo');

cv['onRuntimeInitialized']=()=>{
    let cap = new cv.VideoCapture(video);
    let greenLower = new cv.Scalar(29, 86, 6);
    let greenUpper = new cv.Scalar(64, 255, 255);

    function processFrame() {
        let src = new cv.Mat(video.height, video.width, cv.CV_8UC4);
        let dst = new cv.Mat();
        let mask = new cv.Mat();
        let contours = new cv.MatVector();
        let hierarchy = new cv.Mat();
        cap.read(src);

        let ksize = new cv.Size(11,11);
        cv.GaussianBlur(src, dst, ksize, 0, 0, cv.BORDER_DEFAULT);
        cv.cvtColor(dst, dst, cv.COLOR_BGR2HSV);

        let low = new cv.Mat(dst.rows, dst.cols, dst.type(), greenLower);
        let high = new cv.Mat(dst.rows, dst.cols, dst.type(), greenUpper);

        cv.inRange(dst, low, high, mask);
        cv.erode(mask, mask, new cv.Mat(), new cv.Point(-1, -1), 3);
        cv.dilate(mask, mask, new cv.Mat(), new cv.Point(-1, -1), 3);

        cv.findContours(mask, contours, hierarchy, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE);

        let color = new cv.Scalar(255, 30, 30, 255);
    //    cv.drawContours(src, contours, -1, color, 1, cv.LINE_8, hierarchy, 100);
        for (let i = 0; i < contours.size(); ++i) {
            let cnt = contours.get(i);
            let br = cv.boundingRect(cnt);
            cv.rectangle(src, new cv.Point(br.x, br.y), new cv.Point(br.x + br.width, br.y + br.height), color, 1);
        }

        cv.imshow('opencv', src);
        src.delete();
        dst.delete();
        mask.delete();
        contours.delete();
        hierarchy.delete();
        low.delete();
        high.delete();

        setTimeout(processFrame, 100);
    }
    setTimeout(processFrame, 0);
}

