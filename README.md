# Localization
Implementation of filters for Mobile Robot Localization 

## Usage

- Install the requirements using
```bash
sudo pip install -r requirements.txt
```
- Run Kalman Filter as follows
```bash
$ python main.py --filter Kalman
```
- For the complete list of options, run
```bash
$ python main.py --help
```

## Results
### Running Kalman filter on 1D
![](imgs/original_path.png)
**Fig 1**: Original path of motion

![](imgs/Kalman_tracking.png)
**Fig 2**: Comparison of motion from different view points. The filter used to smoothen the data is done by Savitzky-Golay filter by fitting a 3rd degree polynomial on a window of size 20.

![](imgs/distribution_comparison.gif)
**Fig 3**: Comparison of data distribution as a function of time of flight of motion. 

## To-do
- [x] Kalman Filter
- [x] Particle Filter
- [ ] Histogram Filter
- [ ] Benchmarking the filters

