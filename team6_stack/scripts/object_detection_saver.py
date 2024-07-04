#!/usr/bin/env python
import rospy

import os
import csv
import time
import glob
import numpy as np
import pandas as pd
#from sklearn.cluster import DBSCAN

from std_msgs.msg import Header
from object_detection_msgs.msg import ObjectDetectionInfoArray

timestr = ''

def callback(msg):
    infos = []
    for info in msg.info:
        info = {
            'class_id': info.class_id,
            'id': info.id,
            'x': info.position.x, # in camera frame
            'y': info.position.y, # in camera frame
            'z': info.position.z # in camera frame
        }
        infos.append(info)
    
    print(infos)
    
    # Check if the CSV file exists
    dirname = os.path.dirname(__file__)
    #timestr = time.strftime("%Y%m%d-%H%M%S")
    file_name = 'artifacts-' + timestr + '.csv' # Add the already set timestamp
    #file_name = 'artifacts.csv'
    file_path = os.path.join(dirname, './../data/' + file_name) # Relative path
    file_exists = os.path.isfile(file_path)
    
    # Save the detected artifacts to a CSV file
    if file_exists:
        with open(file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            for info in infos:
                writer.writerow([info['class_id'], info['id'], info['x'], info['y'], info['z']])

# Custom function to find clusters within 2 meters
def find_clusters(positions, radius=2.0):
    clusters = []
    for i, pos in enumerate(positions):
        found_cluster = False
        for cluster in clusters:
            if np.linalg.norm(pos - cluster['center']) <= radius:
                cluster['points'].append(pos)
                cluster['center'] = np.mean(cluster['points'], axis=0)
                found_cluster = True
                break
        if not found_cluster:
            clusters.append({'center': pos, 'points': [pos]})
    return clusters

def shutdown_hook():
    print("Shutting down node and performing cleanup.")

    # Take the latest saved CSV file
    dirname = os.path.dirname(__file__)
    file_path = os.path.join(dirname, './../data/')
    list_of_files = glob.glob(os.path.join(file_path, "*.csv"))
    if list_of_files:
        latest_file = max(list_of_files, key=os.path.getctime)
        print(f"Processing the last saved CSV file: {latest_file}")

        # Final filtering and clustering

        # Load the CSV file into a pandas DataFrame
        df = pd.read_csv(latest_file)

        # METHOD 1: Avg
        # Calculate the average of 'x', 'y', and 'z' for each 'class_id' and 'id' combination
        #avg_df = df.groupby(['class_id', 'id']).agg({'x': 'mean', 'y': 'mean', 'z': 'mean'}).reset_index()

        # METHOD 2: Range + Avg (using sklearn)

    #    # Initialize the DBSCAN clustering algorithm
    #    db = DBSCAN(eps=2, min_samples=1, metric='euclidean')
    #
    #    # Group by class_id and apply DBSCAN clustering within each group
    #    clustered_data = []
    #    for class_id, group in df.groupby('class_id'):
    #        positions = group[['x', 'y', 'z']].values
    #        db.fit(positions)
    #        group['cluster_id'] = db.labels_
    #        
    #        # Calculate the average of 'x', 'y', and 'z' for each cluster
    #        for cluster_id, cluster_group in group.groupby('cluster_id'):
    #            avg_x = cluster_group['x'].mean()
    #            avg_y = cluster_group['y'].mean()
    #            avg_z = cluster_group['z'].mean()
    #            clustered_data.append([class_id, cluster_id, avg_x, avg_y, avg_z])
    #    
    #    # Create a DataFrame from the clustered data
    #    avg_df = pd.DataFrame(clustered_data, columns=['class_id', 'cluster_id', 'x', 'y', 'z'])
        
        # METHOD 3: Range + Avg (using numpy)

        clustered_data = []
        for class_id, group in df.groupby('class_id'):
            positions = group[['x', 'y', 'z']].values
            clusters = find_clusters(positions, radius=2.0)
            
            for i, cluster in enumerate(clusters):
                avg_x, avg_y, avg_z = np.mean(cluster['points'], axis=0)
                clustered_data.append([class_id, i, avg_x, avg_y, avg_z])
        
        # Create a DataFrame from the clustered data
        avg_df = pd.DataFrame(clustered_data, columns=['class_id', 'cluster_id', 'x', 'y', 'z'])

        # Save the filtered data to a new CSV file
        file_name = 'artifacts-' + timestr + '-clustered.csv' # Add the already set timestamp + being clustered flag
        avg_file_path = os.path.join(file_path, file_name)
        avg_df.to_csv(avg_file_path, index=False)
        
        print(f"Filtered data saved to: {avg_file_path}")

def object_detection_saver():
    rospy.init_node('object_detection_saver', anonymous=True)

    global timestr
    
    # Check if the CSV file exists
    dirname = os.path.dirname(__file__)
    timestr = time.strftime("%Y%m%d-%H%M%S")
    #file_name = 'artifacts.csv'
    file_name = 'artifacts-' + timestr + '.csv' # Add timestamp for the beginning of the run
    file_path = os.path.join(dirname, './../data/' + file_name) # Relative path
    file_exists = os.path.isfile(file_path)

    # Write the header only if the file doesn't exist
    if not file_exists:
        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['class_id', 'id', 'x', 'y', 'z'])

    # Register the shutdown hook
    rospy.on_shutdown(shutdown_hook)

    # Subscribe to topics
    #rospy.Subscriber('/object_detector/detection_info', ObjectDetectionInfoArray, callback) # For independent testing
    rospy.Subscriber('/object_inspector/unique_artifacts', ObjectDetectionInfoArray, callback)

    # Keep the script running
    rospy.spin()

if __name__ == '__main__':
    object_detection_saver()
