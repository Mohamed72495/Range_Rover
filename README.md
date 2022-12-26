# Rover_Project
A rover perception and navigation project for detecting, locating and navigating sample rocks from a mars inspired simulated environment utilizing OpenCV and search algorithms.
Quick Look at the Data
There's some example data provided in the test_dataset folder. This basic dataset is enough to get you up and running but if you want to hone your methods more carefully you should record some data of your own to sample various scenarios in the simulator.

Next, read in and display a random image from the test_dataset folder





path = '../test_dataset/IMG/*'
img_list = glob.glob(path)
# Grab a random image and display it
idx = np.random.randint(0, len(img_list)-1)
image = mpimg.imread(img_list[idx])

plt.imshow(image)


