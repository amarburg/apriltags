
Testing against "Debug" of OpenCV master (approx ver 3.0).  
"Debug" build from cmake for AprilTags.  
It's under OS X, so no OpenMP
Macbook Pro 2.6 GHz Core i5


After some initial rearranging, creating the perf and unit tests, and trying to sanitize
the debugging image output:

For 5 reps on a 2592 x 3872 image with 80 tags/image
Mean: 6413 ms, stddev: 245.582

After an initial porting to OpenCV (commit 0a09ec9), I get:

For 5 reps on a 2592 x 3872 image, with an average of 80 tags per image.
Mean: 4631 ms, stddev: 277.693m

Though I did disable m_UseHybrid by default.  That might be saving some time
as well.

For comparison, at the same commit, with Release build I get:

For 5 reps on a 2592 x 3872 image, with an average of 80 tags per image.
Mean: 1237.6 ms, stddev: 193.922

Still pretty slow.
