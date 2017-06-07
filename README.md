# Server-side Face Morphing Engine

## SERVER (C++)

A server-side face morphing engine implementation by Delaunay Triangulation based in C++ with [OpenFace](https://github.com/TadasBaltrusaitis/OpenFace), [OpenCV](https://github.com/opencv/opencv), and [BOOST](https://github.com/boostorg/). This project has been started as a forked project from [HERE](https://github.com/DevendraPratapYadav/FaceMorphing).

### DEPENDENCIES

- OpenFace for Face Detection (FD) & Face Landmark Detection (FLD)
- BOOST for TCP/IP communication w/ some utilities
- OPENCV 3.1 for image processing
- TBB as it's dependent to the OpenFace project
- CMAKE >= 2.8 to compile

### SUPPORTED TYPES OF MORPHING

#### 1. Composable-type Morphing

#### 2. Progressive-type Morphing

### Usage

```
$ ./face-morph-server 9876 .
```

## CLIENT (PHP)

A FLDM (Face Landmark Detection & Morphing) client script, which queries TCP-request to the FLDM server which performs the "face morphing" task to the given images (source and destination) in the command-line shell using PHP script.

### Usage
```
$ /usr/bin/php fldm-client.php ../../sample/1.jpg ../../sample/2.jpg composable 5
```

