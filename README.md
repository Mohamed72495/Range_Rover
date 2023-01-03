# Rover_Project

# NASA Mars Sample & Return Rover (MARS MSR)

## Installation
- Download Project Source code 
- Download Dependencies 
- Open new Terminal in src code file

```python
python drive_rover.py
```
-Open simulator 


## To Test file
- Open new Terminal in src code file

```python
conda activate condaEnv
```
```python
jupyter notebook
```
- Then open Test file



A rover perception and navigation project for detecting, locating and navigating sample rocks from a mars inspired simulated environment utilizing OpenCV and search algorithms.
Quick Look at the Data
There's some example data provided in the test_dataset folder. This basic dataset is enough to get you up and running but if you want to hone your methods more carefully you should record some data of your own to sample various scenarios in the simulator.

Next, read in and display a random image from the test_dataset folder




# Quick Look at the Data
```python
path = '../test_dataset/IMG/*'
img_list = glob.glob(path)
```

# Grab a random image and display it
```python
idx = np.random.randint(0, len(img_list)-1)
image = mpimg.imread(img_list[idx])
plt.imshow(image)
```


