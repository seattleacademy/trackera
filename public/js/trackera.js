(function(window, document) {
    'use strict';

    var Trackera = function Trackera(cockpit) {
        var trackera = this;
        console.log('Loading Trackera plugin');

        this.cockpit = cockpit;
        $('#controls').append('<input type="button" id="track" value="trackme">');

        this.frameWidth = 640;
        this.frameHeight = 360;

        this.newPyramid = new jsfeat.pyramid_t(3);
        this.oldPyramid = new jsfeat.pyramid_t(3);
        this.point_status = new Uint8Array(1);
        this.oldXY = new Float32Array(2);
        this.newXY = new Float32Array(2);
        this.oldRoundedX = 0;
        this.oldRoundedY = 0;

        this.canvas = document.querySelector('#dronestream canvas');
        if (!this.canvas) {
            console.error('Did not find required dronestream canvas');
            return;
        }
        console.log('found canvas, width/height:', this.canvas.clientWidth, this.canvas.clientHeight);

        // add click-handler
        $('#cockpit').append('<div id="trackera"></div>');
        this.div = $('#trackera').get(0);
        this.div.addEventListener('click', function(event) {
            trackera.setTrackingCoords(event.offsetX, event.offsetY);
        });

        this.enabled = false;
        this.observers = {};
        this.locked = false;

        $('#cockpit').append('<img id="trackera-crosshairs" src="/plugin/trackera/img/sniper.png">');
        this.crosshairs = $('#trackera-crosshairs').get(0);
        this.crosshairs.style.display = 'none';

        this.on('points', function(data) {
            console.log("data", data[0].x, trackera.canvas.clientWidth);
            var cCenter = trackera.canvas.clientWidth;
            var gap = 20;
            var newspeed = 0;
            if (data[0].x < cCenter - gap) {
                console.log("turn left");
                newspeed = Math.abs((data[0].x- cCenter)/cCenter);
                trackera.cockpit.socket.emit("/pilot/move", {
                    action: 'counterClockwise',
                    speed: newspeed
                });
            } else
            if (data[0].x > w / 2 + gap) {
                console.log("turn right");
                newspeed = Math.abs((data[0].x- cCenter)/cCenter);
                trackera.cockpit.socket.emit("/pilot/move", {
                    action: 'clockwise',
                    speed: newspeed
                });
            } else {
                trackera.cockpit.socket.emit("/pilot/drone", {
                    action: 'stop'
                });
                console.log("don't move");
            }
            trackera.crosshairs.style.left = (data[0].x - 83) + 'px';
            trackera.crosshairs.style.top = (data[0].y - 83) + 'px';
            trackera.crosshairs.style.display = 'block';
        });

        this.on('locked', function() {
            console.log('target acquired');
        });

        this.on('lost', function() {
            console.log('target lost');
            trackera.cockpit.socket.emit("/pilot/drone", {
                action: 'stop'
            });
            trackera.crosshairs.style.display = 'none';
            trackera.disable();
        });
    };

    Trackera.prototype.prepareTrackingBuffer = function() {
        this.newPyramid.allocate(
            this.frameWidth,
            this.frameHeight,
            jsfeat.U8_t | jsfeat.C1_t
        );
        this.oldPyramid.allocate(
            this.frameWidth,
            this.frameHeight,
            jsfeat.U8_t | jsfeat.C1_t
        );
    };

    Trackera.prototype.update = function(frameBuffer) {
        var tmpXY,
            tmpPyramid,
            roundedX,
            roundedY;

        if (true !== this.enabled) {
            return;
        }
        tmpXY = this.newXY;
        this.newXY = this.oldXY;
        this.oldXY = tmpXY;

        tmpPyramid = this.newPyramid;
        this.newPyramid = this.oldPyramid;
        this.oldPyramid = tmpPyramid;

        this.trackFlow(frameBuffer);

        if (this.point_status[0] == 1) {
            roundedX = Math.round(
                this.newXY[0] * this.canvas.clientWidth / this.frameWidth
            );
            roundedY = Math.round(
                this.newXY[1] * this.canvas.clientHeight / this.frameHeight
            );
            if (
                (!this.locked) ||
                (roundedX !== this.oldRoundedX) ||
                (roundedY !== this.oldRoundedY)
            ) {
                this.oldRoundedX = roundedX;
                this.oldRoundedY = roundedY;
                this.emit('points', [{
                    x: roundedX,
                    y: roundedY
                }]);
            }
            if (!this.locked) {
                this.emit('locked');
            }
            this.locked = true;
        } else {
            if (this.locked) {
                this.emit('lost');
            }
            this.locked = false;
        }
        this.emit('done');
    };

    Trackera.prototype.setTrackingCoords = function(x, y) {
        this.locked = false;

        // translate from (stretched) canvas to framebuffer dimensions:
        this.newXY[0] = x * this.frameWidth / this.canvas.clientWidth;
        this.newXY[1] = y * this.frameHeight / this.canvas.clientHeight;
        // console.log('New tracking coords:', [x,y], this.newXY);
        this.enable();
    };

    Trackera.prototype.trackFlow = function(frameBuffer) {
        this.newPyramid.data[0].data.set(frameBuffer);

        jsfeat.imgproc.equalize_histogram(
            this.newPyramid.data[0].data,
            this.newPyramid.data[0].data
        );

        this.newPyramid.build(this.newPyramid.data[0], true);

        jsfeat.optical_flow_lk.track(
            this.oldPyramid, this.newPyramid,
            this.oldXY, this.newXY,
            1,
            50, // win_size
            30, // max_iterations
            this.point_status,
            0.01, // epsilon,
            0.001 // min_eigen
        );
    };

    Trackera.prototype.enable = function() {
        var trackera = this;
        if (this.enabled) {
            return;
        }
        this.enabled = true;

        if (!this.cockpit.videostream) {
            console.error('The Trackera plugin depends on plugin video-stream');
            return;
        }
        this.prepareTrackingBuffer();

        this.hookNextFrame();
        this.on('done', this.hookNextFrame.bind(this));
    };

    Trackera.prototype.disable = function() {
        this.enabled = false;
    };

    Trackera.prototype.on = function(event, callback) {
        var i = 0,
            handler;
        if (!this.observers[event]) {
            this.observers[event] = [];
        }
        this.observers[event].push(callback);
    };

    Trackera.prototype.emit = function(event, data) {
        var i = 0,
            handler;
        if (this.observers[event]) {
            for (i = 0; handler = this.observers[event][i]; ++i) {
                handler(data);
            }
        }
    };

    Trackera.prototype.hookNextFrame = function() {
        this.cockpit.videostream.onNextFrame(this.update.bind(this));

    };

    window.Cockpit.plugins.push(Trackera);

}(window, document));
