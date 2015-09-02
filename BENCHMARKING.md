
After some initial rearranging, creating the perf and unit tests, and trying to sanitize
the debugging image output:

For 5 reps on a 2592 x 3872 image with 80 tags/image
Mean: 6413 ms, stddev: 245.582
On a Macbook Pro 2.6 GHz Core i5

After an initial porting to OpenCV, I get:

For 5 reps on a 2592 x 3872 image, with an average of 80 tags per image.
Mean: 16591.4 ms, stddev: 927.795

Wow.
