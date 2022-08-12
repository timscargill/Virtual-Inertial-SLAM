#Script to characterize visual and inertial input data in a VI-SLAM dataset, and calculate correlations with trajectory estimate error
#--------------------------------------------------------------------------------------------------------------------------------------
import os
import pandas as pd
import glob
import numpy as np
import cv2
import skimage.measure
import numpy.ma as ma
import math
import statistics
from astropy import stats
import matplotlib.pyplot as plt
from scipy.stats import spearmanr
from scipy.stats import chi2
from scipy.stats import lognorm
import time

# Returns element closest to target in array (used for sub-trajectory analysis)
def findClosest(arr, n, target):
 
    # Corner cases
    if (target <= arr[0]):
        return arr[0]
    if (target >= arr[n - 1]):
        return arr[n - 1]
 
    # Doing binary search
    i = 0; j = n; mid = 0
    while (i < j):
        mid = (i + j) // 2
 
        if (arr[mid] == target):
            return arr[mid]
 
        # If target is less than array
        # element, then search in left
        if (target < arr[mid]) :
 
            # If target is greater than previous
            # to mid, return closest of two
            if (mid > 0 and target > arr[mid - 1]):
                return getClosest(arr[mid - 1], arr[mid], target)
 
            # Repeat for left half
            j = mid
         
        # If target is greater than mid
        else :
            if (mid < n - 1 and target < arr[mid + 1]):
                return getClosest(arr[mid], arr[mid + 1], target)
                 
            # update i
            i = mid + 1
         
    # Only single element left after search
    return arr[mid]
 
 
def getClosest(val1, val2, target):
 
    if (target - val1 >= val2 - target):
        return val2
    else:
        return val1


start_time = time.time()
#--------------------------------------------------------------
#Results analysis
#--------------------------------------------------------------
#User-defined parameters
#------------------------
#Location of experiment folder
EnvSimPath = "D:\\SharedData\\"
#name of sequence
seq_name = "A1"
#name of experiment folder (e.g. 'Sequence_Environment')
env_name = "A1_Name"
#------------------------

#Set approximate frame when IMU initializes (correlations calculated from data after this point)
initialization_frame = 100
#Number of different environment settings (e.g. light levels)
num_settings = 10
#Number of trials per environment setting
num_trials = 10
num_runs =  num_settings * num_trials

#get results for all valid runs
results_csv = EnvSimPath + env_name + "\\Results\\results.csv"

data_results = pd.read_csv(results_csv).values
successful_runs = data_results[:, 0]
ate = data_results[:, 1]
re1 = data_results[:, 2]
re2 = data_results[:, 3]
re5 = data_results[:, 4]
re10 = data_results[:, 5]

#get failed runs
failed_runs = []
for f in range (1,101):
    if f not in successful_runs:
        failed_runs.append(f)

#mask failed runs
runs = np.arange(1, num_runs+1, 1, dtype=int)
runs = np.reshape(runs, (-1, 10))

failures_mask = np.zeros(100)
for f in failed_runs:
    failures_mask[f-1] = 1
valid_runs = np.ma.masked_array(runs, mask=failures_mask)
valid_runs = np.reshape(valid_runs, (-1, 10))

#set up arrays for summary
light_intensity = np.array(["50", "100", "200", "300", "400", "550", "750", "1000", "2500", "5000"])
ate_median = np.zeros(num_settings)
re1_median = np.zeros(num_settings)
re2_median = np.zeros(num_settings)
re5_median = np.zeros(num_settings)
re10_median = np.zeros(num_settings)
failures = np.zeros(num_settings)
errors_5cm = np.zeros(num_settings)
seq_brightness = np.zeros(num_settings)
seq_contrast = np.zeros(num_settings)
seq_entropy = np.zeros(num_settings)
seq_laplacian = np.zeros(num_settings)
seq_corners = np.zeros(num_settings)
seq_inliers = np.zeros(num_settings)
seq_inliers_cov_q = np.zeros(num_settings)
seq_inliers_cov_db = np.zeros(num_settings)
seq_match_orientation_var = np.zeros(num_settings)

#---------------------------------------------------------------------------------------------------------

#characterize inertial data
imu_data = np.genfromtxt(EnvSimPath + env_name + "\\InputData\\" + seq_name + "\\mav0\\imu0\\data.csv",delimiter=',',skip_header=1)
readings = np.arange(1,len(imu_data)+1)

imu_timestamps = imu_data[:,0]

acc_x = imu_data[:,4]
acc_y = imu_data[:,5]
acc_z = imu_data[:,6]

# plotting
plt.title(seq_name + ": Acc")
plt.xlabel("Reading number")
plt.ylabel("Acceleration (m/s^2)")
plt.plot(readings, acc_x, color="red", label="x")
plt.plot(readings, acc_y, color="green", label="y")
plt.plot(readings, acc_z, color="blue", label="z")
plt.show()

seq_acc_mean_x = np.mean(acc_x)
seq_acc_mean_y = np.mean(acc_y)
seq_acc_mean_z = np.mean(acc_z)

mag_sum = 0
dif_sum_x = 0
dif_sum_y = 0
dif_sum_z = 0

for i in range(len(acc_x)):
    mag_sum += (np.sqrt(acc_x[i]**2 + acc_y[i]**2 + acc_z[i]**2))
    dif_sum_x += abs(acc_x[i] - np.mean(acc_x))
    dif_sum_y += abs(acc_y[i] - np.mean(acc_y))
    dif_sum_z += abs(acc_z[i] - np.mean(acc_z))
    
seq_acc_magnitude = mag_sum / len(acc_x)
seq_acc_avg_x_dif = dif_sum_x / len(acc_x)
seq_acc_avg_y_dif = dif_sum_y / len(acc_y)
seq_acc_avg_z_dif = dif_sum_z / len(acc_z)

gyro_x = imu_data[:,1]
gyro_y = imu_data[:,2]
gyro_z = imu_data[:,3]

# plotting
plt.title(seq_name + ": Gyro")
plt.xlabel("Reading number")
plt.ylabel("Rate of rotation (rad/s)")
plt.plot(readings, gyro_x, color="red", label="x")
plt.plot(readings, gyro_y, color="green", label="y")
plt.plot(readings, gyro_z, color="blue", label="z")
plt.show()

seq_gyro_mean_x = np.mean(gyro_x)
seq_gyro_mean_y = np.mean(gyro_y)
seq_gyro_mean_z = np.mean(gyro_z)

mag_sum = 0
dif_sum_x = 0
dif_sum_y = 0
dif_sum_z = 0

gyro_abs_sum = np.zeros(len(gyro_x))

for j in range(len(gyro_x)):
    gyro_abs_sum[j] = abs(np.mean(gyro_x[j])) + abs(np.mean(gyro_y[j])) + abs(np.mean(gyro_z[j]))
    mag_sum += (np.sqrt(gyro_x[j]**2 + gyro_y[j]**2 + gyro_z[j]**2))
    dif_sum_x += abs(gyro_x[j] - np.mean(gyro_x))
    dif_sum_y += abs(gyro_y[j] - np.mean(gyro_y))
    dif_sum_z += abs(gyro_z[j] - np.mean(gyro_z))

seq_gyro_magnitude = mag_sum / len(gyro_x)
seq_gyro_avg_x_dif = dif_sum_x / len(gyro_x)
seq_gyro_avg_y_dif = dif_sum_y / len(gyro_y)
seq_gyro_avg_z_dif = dif_sum_z / len(gyro_z)

#---------------------------------------------------------------------

#loop through 10 dif light intensities to characterize images
for setting in range(0, 10):

    plotted = 0 #indicates if we've plotted for this light intensity
    error_all_trials = np.zeros(0)  #reset all trials array
    
    #Specify image dataset
    image_dataset = EnvSimPath + env_name + "\\InputData\\" + seq_name + "_" + str(setting) + "\\mav0\\cam0\\data"
    
    #Get list of image timestamps
    os.chdir(image_dataset)
    image_files = os.listdir()
    image_files = sorted(image_files)
    images = []
    for file in image_files:
        image = file.replace(".png","")
        images.append(image)
    images = np.array(images).astype('float')
    sequence_length = len(images)
    
    #characterize arrays
    image_brightness_array = np.zeros(sequence_length)
    image_contrast_array = np.zeros(sequence_length)
    image_entropy_array = np.zeros(sequence_length)
    image_laplacian_array = np.zeros(sequence_length)
    image_corners_array = np.zeros(sequence_length)
    
    #loop through images in sequence and characterize
    for i in range(0, sequence_length):
        #progress
        print("Characterizing Setting: " + str(setting) + ", Image: " + str(i))
        #read in image
        img = cv2.imread(image_files[i])
        img = img.astype('uint8')
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h,w = img.shape
        image_brightness_array[i] = (np.mean(img)) / 255
        image_contrast_array[i] = img.std()
        image_entropy_array[i] = skimage.measure.shannon_entropy(img)
        image_laplacian_array[i] = cv2.Laplacian(img, cv2.CV_64F).var()
        fast = cv2.FastFeatureDetector_create()
        image_corners_array[i] = len(fast.detect(img,None))
        
    #----------Feature matching metrics (inliers)-------------------
    image_inliers_array = np.zeros(sequence_length-1)
    image_inliers_cov_q_array = np.zeros(sequence_length-1)
    image_inliers_cov_db_array = np.zeros(sequence_length-1)
    image_match_orientation_var_array = []

    for i in range(1, sequence_length):
        #progress
        print("Characterizing Setting: " + str(setting) + ", Image Pair: " + str(i))
        #read in image pair
        img0 = cv2.imread(image_files[i-1])
        img1 = cv2.imread(image_files[i])
        img0 = img0.astype('uint8')
        img1 = img1.astype('uint8')
        img0 = cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)
        img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
        
        # Initiate orb detector
        orb = cv2.ORB_create()
        
        # find the keypoints and descriptors with fast and brisk
        kp1, des1 = orb.detectAndCompute(img1,None)
        kp0, des0 = orb.detectAndCompute(img0,None)
        
        # BFMatcher with default params
        bf = cv2.BFMatcher()
        
        #if there are keypoints do feature matching
        if (len(kp1) > 1 and len(kp0) > 1):
            matches = bf.knnMatch(des1,des0, k=2)  
            # store all the good matches as per Lowe's ratio test.
            good = []
            for m,n in matches:
                if m.distance < 0.7*n.distance:
                    good.append(m)
        
            if len(good)>5:
                src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
                dst_pts = np.float32([ kp0[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
                
                #RANSAC set to 1.0 because this value in VINS-Mono
                M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,1.0)
                matchesMask = mask.ravel().tolist()
                
                num_inliers = np.sum(matchesMask)
                image_inliers_array[i-1] = num_inliers
                #print(num_inliers)
                angles = np.zeros(num_inliers)
                
                #Calculate Match Quality
                inliers_mask = np.column_stack((1-mask,1-mask))
                
                src_inliers = ma.masked_array(src_pts, inliers_mask)
                dst_inliers = ma.masked_array(dst_pts, inliers_mask)
                
                src_inliers = np.reshape(src_inliers.compressed(), (-1, 2))
                dst_inliers = np.reshape(dst_inliers.compressed(), (-1, 2))
                
                #Inlier Coverage  
                img_q = cv2.cvtColor(img1,cv2.COLOR_GRAY2BGR)
                for s in src_inliers:
                    start_point = (int(s[0] - 60), int(s[1] - 45))
                    end_point = (int(s[0] + 60), int(s[1] + 45))
                    img_q = cv2.rectangle(img_q, start_point, end_point, (0,0,255), -1)
                
                img_db = cv2.cvtColor(img0,cv2.COLOR_GRAY2BGR)
                for d in dst_inliers:
                    start_point = (int(d[0] - 60), int(d[1] - 45))
                    end_point = (int(d[0] + 60), int(d[1] + 45))
                    img_db = cv2.rectangle(img_db, start_point, end_point, (0,0,255), -1)
                    
                covered_pixels_q = np.count_nonzero((img_q == [0, 0, 255]).all(axis = 2))
                covered_pixels_db = np.count_nonzero((img_db == [0, 0, 255]).all(axis = 2))
                coverage_q = covered_pixels_q / (w*h)
                coverage_db = covered_pixels_db / (w*h)
                image_inliers_cov_q_array[i-1] = coverage_q
                image_inliers_cov_db_array[i-1] = coverage_db             
                
                #Match Quality 
                for j in range(len(src_inliers)):
                    opp = dst_inliers[j,1] - src_inliers[j,1]
                    adj = (w - src_inliers[j,0]) + (dst_inliers[j,0])
                    angle = math.degrees(math.atan(opp/adj))      
                    angles[j] = angle
        
                
                matches_orientation_var = np.nanvar(angles)
                if (math.isnan(matches_orientation_var)):
                    matches_orientation_var = 0
                image_match_orientation_var_array.append(matches_orientation_var)
                
            else:
                #print("Not enough matches found")
                image_inliers_array[i-1] = 0
                image_inliers_cov_q_array[i-1] = 0
                image_inliers_cov_db_array[i-1] = 0

        else:
            #print("No matches found")
            image_inliers_array[i-1] = 0
            image_inliers_cov_q_array[i-1] = 0
            image_inliers_cov_db_array[i-1] = 0
    
    #get median of results for summary
    ate_values = []
    re1_values = []
    re2_values = []
    re5_values = []
    re10_values = []
    for r in valid_runs[:,setting].compressed():
        rows = np.where(data_results[:, 0] == r)[0]
        for row in rows:
            ate_values.append(ate[row])
            re1_values.append(re1[row])
            re2_values.append(re2[row])
            re5_values.append(re5[row])
            re10_values.append(re10[row])
    
    ate_median[setting] = np.nanmedian(ate_values)
    re1_median[setting] = np.nanmedian(re1_values)
    re2_median[setting] = np.nanmedian(re2_values)
    re5_median[setting] = np.nanmedian(re5_values)
    re10_median[setting] = np.nanmedian(re10_values)
    failures[setting] = 10 - len(valid_runs[:,setting].compressed())
    
    #add results to arrays for boxplot
    if (setting == 0):
        re2_results_0 = re2_values
    elif (setting == 1):
        re2_results_1 = re2_values
    elif (setting == 2):
        re2_results_2 = re2_values
    elif (setting == 3):
        re2_results_3 = re2_values
    elif (setting == 4):
        re2_results_4 = re2_values
    elif (setting == 5):
        re2_results_5 = re2_values
    elif (setting == 6):
        re2_results_6 = re2_values
    elif (setting == 7):
        re2_results_7 = re2_values
    elif (setting == 8):
        re2_results_8 = re2_values
    elif (setting == 9):
        re2_results_9 = re2_values
    
    #get mean of characterization for summary (whole sequence)
    #individual image metrics
    seq_brightness[setting] = np.nanmean(image_brightness_array)
    seq_contrast[setting] = np.nanmean(image_contrast_array)
    seq_entropy[setting] = np.nanmean(image_entropy_array)
    seq_laplacian[setting] = np.nanmean(image_laplacian_array)
    seq_corners[setting] = np.nanmean(image_corners_array)
    #inlier/match-based metrics        
    seq_inliers[setting] = np.mean(image_inliers_array)
    seq_inliers_cov_q[setting] = np.mean(image_inliers_cov_q_array)
    seq_inliers_cov_db[setting] = np.mean(image_inliers_cov_db_array)
    sufficient_matches = len(image_match_orientation_var_array) / (sequence_length - 1)
    if len(image_match_orientation_var_array) > 0:
        seq_match_orientation_var[setting] = statistics.mean(image_match_orientation_var_array)
    else:
        seq_match_orientation_var[setting] = 0
    
    #--------------------------------------------------------------
    #Sub-trajectory analysis
    #--------------------------------------------------------------
    #Loop through runs
    #Set sub-trajectory length to analyze
    subtrajectory = "2.0"
    error_all_trials = []
    
    for r in valid_runs[:,setting].compressed():       
            
        #print(r)
        #get subtrajectories data
        errors_csv = EnvSimPath + env_name + "\\Results\\sub_errors_" + seq_name + "_" + str(r) + "_" + subtrajectory + ".csv"
        
        if os.path.exists(errors_csv):
            data = pd.read_csv(errors_csv).values
            start_timestamps = data[:, 0]
            end_timestamps = data[:, 1]
            error = data[:, 2]
            for e in error:
                error_all_trials.append(e)
            num_subtrajectories = start_timestamps.size
            
            #set up arrays
            start_images = np.zeros(num_subtrajectories)
            end_images = np.zeros(num_subtrajectories)
            start_image_indexes = np.zeros(num_subtrajectories)
            end_image_indexes = np.zeros(num_subtrajectories)
            num_images = np.zeros(num_subtrajectories)
            mean_brightness = np.zeros(num_subtrajectories)
            mean_contrast = np.zeros(num_subtrajectories)
            mean_entropy = np.zeros(num_subtrajectories)
            mean_laplacian = np.zeros(num_subtrajectories)
            mean_corners = np.zeros(num_subtrajectories)
            min_corners = np.zeros(num_subtrajectories)
            mean_inliers = np.zeros(num_subtrajectories)
            min_inliers = np.zeros(num_subtrajectories)
            mean_inliers_cov_q = np.zeros(num_subtrajectories)
            mean_inliers_cov_db = np.zeros(num_subtrajectories)
            mean_match_orientation_var = np.zeros(num_subtrajectories)
            #inertial
            acc_magnitude = np.zeros(num_subtrajectories)
            gyro_magnitude = np.zeros(num_subtrajectories)
            
            #find closest inertial readings to timestamps
            arr = imu_timestamps
            n = len(imu_timestamps)
            for s in range(0, num_subtrajectories):
                target = float(start_timestamps[s])
                start_imu = findClosest(imu_timestamps, n, target)
                target = float(end_timestamps[s])
                end_imu = findClosest(imu_timestamps, n, target)
                #calculate num readings in subtrajectory
                start_index = int(np.where(imu_timestamps == start_imu)[0])
                start_imu_index = start_index
                end_index = int(np.where(imu_timestamps == end_imu)[0])
                end_imu_index = end_index
                num_readings = end_index - start_index
                #calculate metrics
                for i in range(start_index,end_index):
                    mag_sum += (np.sqrt(acc_x[i]**2 + acc_y[i]**2 + acc_z[i]**2))
                acc_magnitude[s] = mag_sum / num_readings
                mag_sum = 0
                for i in range(start_index,end_index):
                    mag_sum += (np.sqrt(gyro_x[i]**2 + gyro_y[i]**2 + gyro_z[i]**2))
                gyro_magnitude[s] = mag_sum / num_readings
                mag_sum = 0
            
            #find closest images to timestamps
            arr = images
            n = len(images)
            #print(format(match, '.0f'))
            for s in range(0, num_subtrajectories):
                #print("Processing Setting: " + str(setting) + ", Subtrajectory: " + str(s))
                target = float(start_timestamps[s])
                start_images[s] = findClosest(images, n, target)
                target = float(end_timestamps[s])
                end_images[s] = findClosest(images, n, target)
                #calculate num images in subtrajectory
                start_index = int(np.where(images == start_images[s])[0])
                start_image_indexes[s] = start_index
                end_index = int(np.where(images == end_images[s])[0])
                end_image_indexes[s] = end_index
                num_images[s] = end_index - start_index
                #calculate metrics
                mean_brightness[s] = np.nanmean(image_brightness_array[start_index:end_index])
                mean_contrast[s] = np.nanmean(image_contrast_array[start_index:end_index])
                mean_entropy[s] = np.nanmean(image_entropy_array[start_index:end_index])
                mean_laplacian[s] = np.nanmean(image_laplacian_array[start_index:end_index])
                mean_corners[s] = np.nanmean(image_corners_array[start_index:end_index])
                try:
                    min_corners[s] = np.nanmin(image_corners_array[start_index:end_index])
                except ValueError:
                    min_corners[s] = 0
                mean_inliers[s] = np.nanmean(image_inliers_array[start_index:end_index])
                try:
                    min_inliers[s] = np.nanmin(image_inliers_array[start_index:end_index])
                except ValueError:
                    min_inliers[s] = 0
                mean_inliers_cov_q[s] = np.nanmean(image_inliers_cov_q_array[start_index:end_index])
                mean_inliers_cov_db[s] = np.nanmean(image_inliers_cov_db_array[start_index:end_index])
                mean_match_orientation_var[s] = np.nanmean(image_match_orientation_var_array[start_index:end_index])
            
            #plot for first run of new sequence
            if (plotted == 0):
                #Error histogram
                plt.title("Error histogram for subtrajectory length " + subtrajectory + "m (Light Intensity = " + light_intensity[setting] + ") (Single Trial)")
                plt.hist(error, 40, range=[0,0.2], alpha=1, label='Translational error (m)', edgecolor="black")
                plt.xlabel("Translational error (m)")
                plt.ylabel("Frequency")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\sub_error_histogram_" + light_intensity[setting] + "_single.png", bbox_inches='tight')
                plt.show()         
                
                #plot time series
                plt.title("Acc Magnitude for subtrajectory length " + subtrajectory + "m (Light Intensity = " + light_intensity[setting] + ")")
                plt.xlabel("Subtrajectory")
                plt.ylabel("Acc magnitude in subtrajectory")
                ax = plt.gca()
                ax.set_ylim([7, 15])
                plt.plot(acc_magnitude, color="black")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\acc_magnitude_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Gyro Magnitude for subtrajectory length " + subtrajectory + "m (Light Intensity = " + light_intensity[setting] + ")")
                plt.xlabel("Subtrajectory")
                plt.ylabel("Gyro magnitude in subtrajectory")
                ax = plt.gca()
                ax.set_ylim([0, 3])
                plt.plot(gyro_magnitude, color="black")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\gyro_magnitude_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()       
                
                plt.title("Brightness for subtrajectory length " + subtrajectory + "m (Light Intensity = " + light_intensity[setting] + ")")
                plt.xlabel("Subtrajectory")
                plt.ylabel("Mean brightness in subtrajectory")
                plt.plot(mean_brightness, color="blue")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\brightness_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Contrast for subtrajectory length " + subtrajectory + "m (Light Intensity = " + light_intensity[setting] + ")")
                plt.xlabel("Subtrajectory")
                plt.ylabel("Mean contrast in subtrajectory")
                plt.plot(mean_contrast, color="brown")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\contrast_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Entropy for subtrajectory length " + subtrajectory + "m (Light Intensity = " + light_intensity[setting] + ")")
                plt.xlabel("Subtrajectory")
                plt.ylabel("Mean entropy in subtrajectory")
                plt.plot(mean_entropy, color="green")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\entropy_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Laplacian for subtrajectory length " + subtrajectory + "m (Light Intensity = " + light_intensity[setting] + ")")
                plt.xlabel("Subtrajectory")
                plt.ylabel("Mean laplacian in subtrajectory")
                plt.plot(mean_laplacian, color="black")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\laplacian_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Corners for subtrajectory length " + subtrajectory + "m (Light Intensity = " + light_intensity[setting] + ")")
                plt.xlabel("Subtrajectory")
                plt.ylabel("Mean corners in subtrajectory")
                plt.plot(mean_corners, color="purple")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\corners_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Min Corners for subtrajectory length " + subtrajectory + "m (Light Intensity = " + light_intensity[setting] + ")")
                plt.xlabel("Subtrajectory")
                plt.ylabel("Min corners in subtrajectory")
                plt.plot(min_corners, color="olive")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\min_corners_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Inliers for subtrajectory length " + subtrajectory + "m (Light Intensity = " + light_intensity[setting] + ")")
                plt.xlabel("Subtrajectory")
                plt.ylabel("Mean inliers in subtrajectory")
                plt.plot(mean_inliers, color="blue")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\inliers_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Min Inliers for subtrajectory length " + subtrajectory + "m (Light Intensity = " + light_intensity[setting] + ")")
                plt.xlabel("Subtrajectory")
                plt.ylabel("Min inliers in subtrajectory")
                plt.plot(min_inliers, color="cyan")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\min_inliers_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Inliers Coverage for subtrajectory length " + subtrajectory + "m (Light Intensity = " + light_intensity[setting] + ")")
                plt.xlabel("Subtrajectory")
                plt.ylabel("Mean inliers coverage in subtrajectory")
                plt.plot(mean_inliers_cov_q, color="orange")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\inliers_cov_q_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Match Orientation Variance for subtrajectory length " + subtrajectory + "m (Light Intensity = " + light_intensity[setting] + ")")
                plt.xlabel("Subtrajectory")
                plt.ylabel("Mean match orientation variance in subtrajectory")
                plt.plot(mean_match_orientation_var, color="brown")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\match_orientation_var_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Error for subtrajectory length " + subtrajectory + "m (Light Intensity = " + light_intensity[setting] + ")")
                plt.xlabel("Subtrajectory")
                plt.ylabel("Translational error (m) in subtrajectory")
                plt.plot(error, color="red")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\sub_error_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                ##For below scatter and correlations remove first N frames##
                
                #Error vs single metric scatter
                plt.title("Error vs Acc Magnitude (Light Intensity = " + light_intensity[setting] + ", first " + str(initialization_frame) + " frames removed)")
                plt.scatter(acc_magnitude[initialization_frame:], error[initialization_frame:])
                plt.xlabel("Acc Magnitude")
                plt.ylabel("Error")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\acc_magnitude_scatter_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Error vs Gyro Magnitude (Light Intensity = " + light_intensity[setting] + ", first " + str(initialization_frame) + " frames removed)")
                plt.scatter(gyro_magnitude[initialization_frame:], error[initialization_frame:])
                plt.xlabel("Gyro Magnitude")
                plt.ylabel("Error")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\gyro_magnitude_scatter_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Error vs Mean Brightness (Light Intensity = " + light_intensity[setting] + ", first " + str(initialization_frame) + " frames removed)")
                plt.scatter(mean_brightness[initialization_frame:], error[initialization_frame:])
                plt.xlabel("Brightness")
                plt.ylabel("Error")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\brightness_scatter_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Error vs Mean Contrast (Light Intensity = " + light_intensity[setting] + ", first " + str(initialization_frame) + " frames removed)")
                plt.scatter(mean_contrast[initialization_frame:], error[initialization_frame:])
                plt.xlabel("Contrast")
                plt.ylabel("Error")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\contrast_scatter_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Error vs Mean Entropy (Light Intensity = " + light_intensity[setting] + ", first 100 " + str(initialization_frame) + " removed)")
                plt.scatter(mean_entropy[initialization_frame:], error[initialization_frame:])
                plt.xlabel("Entropy")
                plt.ylabel("Error")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\entropy_scatter_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Error vs Mean Laplacian (Light Intensity = " + light_intensity[setting] + ", first " + str(initialization_frame) + " frames removed)")
                plt.scatter(mean_laplacian[initialization_frame:], error[initialization_frame:])
                plt.xlabel("Laplacian")
                plt.ylabel("Error")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\laplacian_scatter_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Error vs Mean Corners (Light Intensity = " + light_intensity[setting] + ", first " + str(initialization_frame) + " frames removed)")
                plt.scatter(mean_corners[initialization_frame:], error[initialization_frame:])
                plt.xlabel("Corners")
                plt.ylabel("Error")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\corners_scatter_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Error vs Min Corners (Light Intensity = " + light_intensity[setting] + ", first " + str(initialization_frame) + " frames removed)")
                plt.scatter(min_corners[initialization_frame:], error[initialization_frame:])
                plt.xlabel("Corners")
                plt.ylabel("Error")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\min_corners_scatter_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Error vs Mean Inliers (Light Intensity = " + light_intensity[setting] + ", first " + str(initialization_frame) + " frames removed)")
                plt.scatter(mean_inliers[initialization_frame:], error[initialization_frame:])
                plt.xlabel("Inliers")
                plt.ylabel("Error")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\corners_scatter_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Error vs Min Inliers (Light Intensity = " + light_intensity[setting] + ", first " + str(initialization_frame) + " frames removed)")
                plt.scatter(min_inliers[initialization_frame:], error[initialization_frame:])
                plt.xlabel("Inliers")
                plt.ylabel("Error")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\min_inliers_scatter_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Error vs Mean Inliers Coverage (Light Intensity = " + light_intensity[setting] + ", first " + str(initialization_frame) + " frames removed)")
                plt.scatter(mean_inliers_cov_q[initialization_frame:], error[initialization_frame:])
                plt.xlabel("Inliers Coverage")
                plt.ylabel("Error")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\inliers_cov_q_scatter_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plt.title("Error vs Mean Match Orientation Variance (Light Intensity = " + light_intensity[setting] + ", first " + str(initialization_frame) + " frames removed)")
                plt.scatter(mean_match_orientation_var[initialization_frame:], error[initialization_frame:])
                plt.xlabel("Match Orientation Variance")
                plt.ylabel("Error")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\match_orientation_var_scatter_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()


                #Metric correlations
                v_spearman_array = np.array([spearmanr(acc_magnitude[initialization_frame:], error[initialization_frame:]), spearmanr(gyro_magnitude[initialization_frame:], error[initialization_frame:]), spearmanr(mean_brightness[initialization_frame:], error[initialization_frame:]), spearmanr(mean_contrast[initialization_frame:], error[initialization_frame:]), spearmanr(mean_entropy[initialization_frame:], error[initialization_frame:]), spearmanr(mean_laplacian[initialization_frame:], error[initialization_frame:]), spearmanr(mean_corners[initialization_frame:], error[initialization_frame:]), spearmanr(min_corners[initialization_frame:], error[initialization_frame:]),  spearmanr(mean_inliers[initialization_frame:], error[initialization_frame:]), spearmanr(min_inliers[initialization_frame:], error[initialization_frame:]),  spearmanr(mean_inliers_cov_q[initialization_frame:], error[initialization_frame:]), spearmanr(mean_match_orientation_var[initialization_frame:], error[initialization_frame:])])
                v_feature_names = np.array(["acc_mag", "gyro_mag", "brightness", "contrast", "entropy", "laplacian", "corners", "min_corners", "inliers", "min_inliers", "inliers_cov_q", "match_orientation_var"])
                v_coef = v_spearman_array[:,0]
                v_p = v_spearman_array[:,1]
                plt.title("Correlations between Error and Input Data Metrics  (Light Intensity = " + light_intensity[setting] + ", first " + str(initialization_frame) + " frames removed)") 
                plt.barh(v_feature_names, v_coef)
                plt.xlabel("Spearman Correlation Coefficient (Error)")
                plt.ylabel("Visual Scene Metric")
                plt.savefig(EnvSimPath + env_name + "\\Analysis\\correlations_" + light_intensity[setting] + ".png", bbox_inches='tight')
                plt.show()
                
                plotted = 1 #indicates if we've plotted for this light intensity
        
            #convert to strings for subtrajectory output
            start_timestamps = ["%.0f" % timestamp for timestamp in start_timestamps]
            end_timestamps = ["%.0f" % timestamp for timestamp in end_timestamps]
            start_images = ["%.0f" % image for image in start_images]
            end_images = ["%.0f" % image for image in end_images]
            num_images = ["%.0f" % image for image in num_images]
            
            #create subtrajectory output csv
            columns_list = np.array(["start_timestamp", "end_timestamp", "error", "start_image", "end_image", "start_image_index", "end_image_index", "num_images", "brightness", "contrast", "entropy", "laplacian", "corners", "min_corners", "inliers", "min_inliers", "inliers_cov_q", "inliers_cov_db", "match_orientation_var"])
            csv_data = np.column_stack((start_timestamps, end_timestamps, error, start_images, end_images, start_image_indexes, end_image_indexes, num_images, mean_brightness, mean_contrast, mean_entropy, mean_laplacian, mean_corners, min_corners, mean_inliers, min_inliers, mean_inliers_cov_q, mean_inliers_cov_db, mean_match_orientation_var))
            output_csv = EnvSimPath + env_name + "\\Results\\results_" + seq_name + "_" + str(r) + "_" + subtrajectory + ".csv"
            pd.DataFrame(csv_data).to_csv(output_csv, header=columns_list, index=False)
            
    #Error all trials histogram
    plt.title("Error histogram for subtrajectory length " + subtrajectory + "m (Light Intensity = " + light_intensity[setting] + ") (All Trials)")
    plt.hist(error_all_trials, 40, range=[0,0.2], alpha=1, label='Translational error (m)', edgecolor="black")
    plt.xlabel("Translational error (m)")
    plt.ylabel("Frequency")
    plt.savefig(EnvSimPath + env_name + "\\Analysis\\sub_error_histogram_" + light_intensity[setting] + "_all.png", bbox_inches='tight')
    plt.show()
    
    #add results to arrays for subtrajectories boxplot
    if (setting == 0):
        re2_subtrajectories_0 = error_all_trials
    elif (setting == 1):
        re2_subtrajectories_1 = error_all_trials
    elif (setting == 2):
        re2_subtrajectories_2 = error_all_trials
    elif (setting == 3):
        re2_subtrajectories_3 = error_all_trials
    elif (setting == 4):
        re2_subtrajectories_4 = error_all_trials
    elif (setting == 5):
        re2_subtrajectories_5 = error_all_trials
    elif (setting == 6):
        re2_subtrajectories_6 = error_all_trials
    elif (setting == 7):
        re2_subtrajectories_7 = error_all_trials
    elif (setting == 8):
        re2_subtrajectories_8 = error_all_trials
    elif (setting == 9):
        re2_subtrajectories_9 = error_all_trials
    
    #Error all trials stats
    if len(error_all_trials) > 0:
        num_errors_5cm = 0
        for e in error_all_trials:
            if e > 0.05:
                num_errors_5cm += 1
         
        errors_5cm[setting] = (num_errors_5cm / len(error_all_trials)) * 100
    else:
        errors_5cm[setting] = 0

#create summary bar chart
plt.bar(range(len(re2_median)), re2_median, align='center', alpha=1)
plt.grid(color='#95a5a6', linestyle='--', linewidth=2, axis='y', alpha=0.5)
plt.xticks(range(len(re2_median)), light_intensity)
plt.xlabel('Light intensity  (lumens)')
plt.ylabel('Median relative error (m)')
plt.title('Median relative error (2m) on ' + seq_name + ' at 10 different light levels (10 trials)')
plt.savefig(EnvSimPath + env_name + "\\Analysis\\re2_bar_chart.png", bbox_inches='tight')
plt.show()

#create summary re2 boxplot
box_plot_data=[re2_results_0,re2_results_1,re2_results_2,re2_results_3,re2_results_4,re2_results_5,re2_results_6,re2_results_7,re2_results_8,re2_results_9]
plt.boxplot(box_plot_data,labels=light_intensity)
plt.xlabel('Light intensity  (lumens)')
plt.ylabel('Relative error (m)')
plt.title('Relative error (2m) on ' + seq_name + ' at 10 different light levels (10 trials)')
plt.savefig(EnvSimPath + env_name + "\\Analysis\\re2_box_plot.png", bbox_inches='tight')
plt.show()

#create summary re2 subtrajectories boxplot
box_plot_data2=[re2_subtrajectories_0,re2_subtrajectories_1,re2_subtrajectories_2,re2_subtrajectories_3,re2_subtrajectories_4,re2_subtrajectories_5,re2_subtrajectories_6,re2_subtrajectories_7,re2_subtrajectories_8,re2_subtrajectories_9]
plt.boxplot(box_plot_data2,labels=light_intensity)
plt.xlabel('Light intensity  (lumens)')
plt.ylabel('Translational error (m)')
plt.title('Subtrajectory (2m) translational error on ' + seq_name + ' at 10 different light levels (10 trials)')
plt.savefig(EnvSimPath + env_name + "\\Analysis\\subtrajectories_box_plot.png", bbox_inches='tight')
plt.show()

#create results summary output csv
summary_columns_list = np.array(["light intensity", "ATE", "RE_1m", "RE_2m", "RE_5m", "RE_10m", "failures", "errors_5cm_%", "brightness", "contrast", "entropy", "laplacian", "corners", "inliers", "inliers_cov_q", "inliers_cov_db", "match_orientation_var" ])
summary_csv_data = np.column_stack((light_intensity, ate_median, re1_median, re2_median, re5_median, re10_median, failures, errors_5cm, seq_brightness, seq_contrast, seq_entropy, seq_laplacian, seq_corners, seq_inliers, seq_inliers_cov_q, seq_inliers_cov_db, seq_match_orientation_var))
summary_output_csv = EnvSimPath + env_name + "\\Analysis\\results_summary.csv"
pd.DataFrame(summary_csv_data).to_csv(summary_output_csv, header=summary_columns_list, index=False)
    
print("--- %s seconds ---" % (time.time() - start_time))
