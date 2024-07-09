# -*- coding: utf-8 -*-
"""
Created on Wed Jul  3 13:39:02 2024

@author: sbamford


RETARD: Recorded Event Time-synchronization Agnostic Representation Dataplayer (Thanks, ChatGpt)

Given a container, create a player with one subwindow for each channel / data type
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider, Button
from bimvee.plotDvsContrast import getEventImageForTimeRange
import math
import time 

from bimvee.visualisers.visualiserDvs import VisualiserDvs
from bimvee.visualisers.visualiserFrame import VisualiserFrame, findNearest

class ViewerDvs(VisualiserDvs):

    def __init__(self, data, ax):
        #self.data = events
        self.ax = ax
        # Making the dimensions instance attributes - TODO: push this improvement down into the visualisers
        self.dimX = data.get('dimX') or np.max(data['x']) + 1
        self.dimY = data.get('dimY') or np.max(data['y']) + 1
        # Currently, the base class requires the data to carry dimX/Y attributes:
        data['dimX'] = self.dimX
        data['dimY'] = self.dimY
        self.contrast = 1
        self.time_window = 0.03 # this default is overwritten by the slider
        self.label = data.get('label', '')
        self.image_type = 'not_polarized'
        super(ViewerDvs, self).__init__(data)
        
    def update(self, target_time):
            
        event_image = self.get_frame(target_time, self.time_window, image_type=self.image_type, contrast=self.contrast) # to do -get the lower function internalise contrast
        '''
        startTime = target_time - self.time_window / 2
        endTime = target_time + self.time_window / 2
        event_image = getEventImageForTimeRange(
            self.events, 
            startTime=startTime, 
            endTime=endTime, 
            contrast=self.contrast,
            dimX=self.dimX,
            dimY=self.dimY,
            image_type='not_polarized')
        event_image += self.contrast
        self.ax.imshow(event_image, cmap='gray', vmin=0, vmax=self.contrast*2)
        '''
        # Remove previous data
        self.ax.clear()
        self.ax.imshow(event_image, cmap='gray', vmin=0, vmax=255)
        self.ax.set_xticks([])
        self.ax.set_xticks([], minor=True)
        self.ax.set_yticks([])
        self.ax.set_yticks([], minor=True)
        self.ax.set_title(self.label)

class ViewerFrame(VisualiserFrame):

    def __init__(self, data, ax):
        #self.data = events
        self.ax = ax
        # Making the dimensions instance attributes - TODO: push this improvement down into the visualisers
        #self.dimX = data.get('dimX') or np.max(data['x']) + 1
        #self.dimY = data.get('dimY') or np.max(data['y']) + 1
        # Currently, the base class requires the data to carry dimX/Y attributes:
        #data['dimX'] = self.dimX
        #data['dimY'] = self.dimY
        #self.contrast = 1
        self.time_window = 0.03 # for frames, need to decide between either showing nearest frame within window, or showing the nearest one in any case
        self.label = data.get('label', '')
        self.image_type = 'not_polarized'
        super(ViewerFrame, self).__init__(data)
        
    def update(self, target_time):
            
        image = self.get_frame(target_time, self.time_window) 
        # Remove previous data
        self.ax.clear()
        self.ax.imshow(image, cmap='gray', vmin=0, vmax=255)
        self.ax.set_xticks([])
        self.ax.set_xticks([], minor=True)
        self.ax.set_yticks([])
        self.ax.set_yticks([], minor=True)
        self.ax.set_title(self.label)



# TODO: This functionality, or some of it, may belong in the container class
def get_dvs_data(container, label=[]):
    keys = container.keys()
    if ('ts' in keys
        and 'pol' in keys
        and 'x' in keys
        and 'y' in keys):
        container['label'] = '_'.join(label)
        container['data_type'] = 'dvs'
        return [container]
        # TODO: put stricter check here for data type
    else:
        dvs_dicts = []
        for key, value in container.items():
            if type(value) == dict:
                dvs_dicts = dvs_dicts + get_dvs_data(value, label + [str(key)])
        return dvs_dicts

def get_frame_data(container, label=[]):
    keys = container.keys()
    if ('ts' in keys
        and 'frames' in keys):
        container['label'] = '_'.join(label)
        container['data_type'] = 'frame'
        return [container]
        # TODO: put stricter check here for data type
    else:
        frame_dicts = []
        for key, value in container.items():
            if type(value) == dict:
                frame_dicts = frame_dicts + get_frame_data(value, label + [str(key)])
        return frame_dicts


from math import log10, floor
def round_to_1_sf(x):
    return round(x, -int(floor(log10(abs(x)))))

class Player():
    
    # TODO: - there might be a global override for local controls like contrast, TBD
    is_playing = True
    interval_ms = 100 # time between animation frames
    viewers = []
    
    def __init__(self, container):
        self.fig = plt.figure()
        dvs_containers = get_dvs_data(container)
        frame_containers = get_frame_data(container)
        all_rendered_containers = dvs_containers + frame_containers
        num_containers = len(all_rendered_containers)
        num_cols = math.ceil(math.sqrt(num_containers))
        num_rows = math.ceil(num_containers / num_cols)
        x_min = 0
        y_min = 0.1
        x_extent = 1
        y_extent = 0.7
        x_spacing = 0.15 / (num_cols + 1)
        y_spacing = 0.15 / (num_rows + 1)
        x_extent_per_col = (x_extent - x_spacing) / num_cols
        y_extent_per_row = (y_extent - y_spacing) / num_rows
        self.last_time = time.time()
        self.speed = 1
        
        for idx, rendered_container in enumerate(all_rendered_containers):
            row_idx = math.floor(idx / num_cols)
            col_idx = idx % num_cols
            ax = self.fig.add_axes([
                x_min + x_spacing / 2 + col_idx / max((num_cols - 1), 1) * x_extent_per_col, 
                y_min + y_spacing / 2 + row_idx / max((num_rows - 1), 1) * y_extent_per_row, 
                x_extent_per_col - x_spacing / 2, 
                y_extent_per_row - y_spacing / 2]) # xmin ymin x-extent y-extent
            if rendered_container['data_type'] == 'dvs':
                viewer = ViewerDvs(rendered_container, ax)
            elif rendered_container['data_type'] == 'frame':
                viewer = ViewerFrame(rendered_container, ax)
            self.viewers.append(viewer)
        
        # recalculations here based on container(s)
        max_times = [rendered_container['ts'][-1] for rendered_container in all_rendered_containers]
        self.max_time = max(max_times)
        
        # Add controls
        self.ax_time_slider = self.fig.add_axes([0.2, 0.825, 0.7, 0.05]) # xmin ymin x-extent y-extent
        self.slider_time = Slider(
            ax=self.ax_time_slider,
            label='time', 
            valmin=0, 
            valmax=self.max_time,
            valinit=0)

        self.ax_speed_slider = self.fig.add_axes([0.2, 0.875, 0.7, 0.05]) # xmin ymin x-extent y-extent
        self.slider_speed = Slider(
            ax=self.ax_speed_slider,
            label='speed', 
            valmin=-3, 
            valmax=1,
            valinit=0)

        self.ax_window_slider = self.fig.add_axes([0.2, 0.925, 0.7, 0.05]) # xmin ymin x-extent y-extent
        self.slider_window = Slider(
            ax=self.ax_window_slider,
            label='time_window', 
            valmin=-3, 
            valmax=3,
            valinit=1.5)

        self.ax_button_play = self.fig.add_axes([0.05, 0.8, 0.05, 0.05]) # xmin ymin x-extent y-extent
        self.button_play = Button(self.ax_button_play, 'Pause') #, color="blue")
        self.button_play.on_clicked(self.toggle_play)
                        
        self.ax_button_pol = self.fig.add_axes([0.05, 0.875, 0.05, 0.05]) # xmin ymin x-extent y-extent
        self.button_pol = Button(self.ax_button_pol, 'Polarity') #, color="blue")
        self.button_pol.on_clicked(self.toggle_pol)
                        
        self.slider_time.on_changed(self.update_viewers)
        self.slider_speed.on_changed(self.slider_speed_manual_control)
        self.slider_speed_manual_control(self.slider_speed.valinit)
        self.slider_window.on_changed(self.slider_window_manual_control)
        self.slider_window_manual_control(self.slider_window.valinit)
        
        #self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        self.ani = animation.FuncAnimation(
            self.fig, 
            self.slider_time_autoplay, 
            interval=self.interval_ms, 
            cache_frame_data=False) #TODO - could cahcing helpo?
        
        self.update_viewers(0)

    def toggle_play(self, val):
        if self.is_playing:
            self.is_playing = False
            self.button_play.label.set_text('Play')
        else:
            self.last_time = time.time()
            self.is_playing = True
            self.button_play.label.set_text('Pause')
                        
    def toggle_pol(self, val):
        for viewer in self.viewers:
            if type(viewer) == ViewerDvs:
                if viewer.image_type == 'not_polarized':
                    viewer.image_type = 'count'
                else:
                    viewer.image_type = 'not_polarized'
                        
    # update a single plot pane - this should be pushed to a visualiser subclass
    def update_viewers(self, val):
        # call for each viewer
        target_time = self.slider_time.val
        for viewer in self.viewers:
            viewer.update(target_time)

    def slider_speed_manual_control(self, val):
        self.speed = round_to_1_sf(10 ** val)
        self.slider_speed.valtext.set_text(self.speed)

    def slider_window_manual_control(self, val):
        self.time_window = round_to_1_sf(10 ** val) / 1000
        for viewer in self.viewers:
            viewer.time_window = self.time_window
        self.slider_window.valtext.set_text(self.time_window)

    def slider_time_autoplay(self, _):
        if self.is_playing:
            interval = (time.time() - self.last_time) * self.speed
            self.last_time = time.time()
            new_time = (self.slider_time.val + interval) % self.slider_time.valmax
            self.slider_time.set_val(new_time)
    
