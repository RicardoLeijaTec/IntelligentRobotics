#!/usr/bin/python
import rospy
from std_msgs.msg import String
import wx
import xml.dom.minidom
from math import pi
from threading import Thread


RANGE = 10000

class CalibrateGui(wx.Frame):
    def __init__(self, title):
        wx.Frame.__init__(self, None, -1, title, (-1, -1))
        self.hsv_map = {}
        self.color_values = {
            'h_min': {'min': 0, 'max': 255, 'value': 0},
            's_min': {'min': 0, 'max': 255, 'value': 0},
            'v_min': {'min': 0, 'max': 255, 'value': 0},
            'h_max': {'min': 0, 'max': 255, 'value': 0},
            's_max': {'min': 0, 'max': 255, 'value': 0},
            'v_max': {'min': 0, 'max': 255, 'value': 0},
        }
        panel = wx.Panel(self, wx.ID_ANY)
        box = wx.BoxSizer(wx.VERTICAL)
        font = wx.Font(9, wx.SWISS, wx.NORMAL, wx.BOLD)
	self.color_publisher = rospy.Publisher("color_publisher", String, queue_size = 10)
	rospy.init_node('image_listener', anonymous = True)
        
        ### Sliders ###
        for name in ['h_min', 's_min', 'v_min', 'h_max', 's_max', 'v_max']:
            color_byte = self.color_values[name]

            if color_byte['min'] == color_byte['max']:
                continue

            row = wx.GridSizer(1, 2, 1)
            label = wx.StaticText(panel, -1, name)
            label.SetFont(font)
            row.Add(label, 1, wx.ALIGN_CENTER_VERTICAL)

            display = wx.TextCtrl (
                panel, value=str(0),
                style=wx.TE_READONLY | wx.ALIGN_RIGHT
            )

            row.Add(display, flag = wx.ALIGN_RIGHT | wx.ALIGN_CENTER_VERTICAL)
            box.Add(row, 1, wx.EXPAND)
            slider = wx.Slider(
                panel, -1, RANGE/2, 0, RANGE, 
                style= wx.SL_AUTOTICKS | wx.SL_HORIZONTAL
            )
            slider.SetFont(font)
            box.Add(slider, 1, wx.EXPAND)

            self.hsv_map[name] = {
                'slidervalue': 0, 'display': display, 
                'slider': slider, 'color_byte': color_byte,
            }

        ### Buttons ###
        self.ctrbutton = wx.Button(panel, 1, 'Center')
        self.Bind(wx.EVT_SLIDER, self.sliderUpdate)
        wx.EVT_BUTTON(self, 1, lambda evt: self.center())
        box.Add(self.ctrbutton, 0, wx.EXPAND)
        
        panel.SetSizer(box)
        self.center()
        box.Fit(self)
        self.update_values()


    def update_values(self):
        for byte_info in self.hsv_map.values():
            purevalue = byte_info['slidervalue']
            color_byte = byte_info['color_byte']
            value = self.sliderToValue(purevalue)
            color_byte['value'] = value
        self.update_sliders()
        self.publish_values()

    def update_sliders(self):
        for byte_info in self.hsv_map.values():
            color_byte = byte_info['color_byte']
            byte_info['slidervalue'] = self.valueToSlider(color_byte['value'])
            byte_info['slider'].SetValue(byte_info['slidervalue'])
            byte_info['display'].SetValue("%.2f" % color_byte['value'])

    def center(self):
        for byte_info in self.hsv_map.values():
            byte_info['slidervalue'] = self.valueToSlider(255//2)
        self.update_values()

    def sliderUpdate(self, evt):
        for byte_info in self.hsv_map.values():
            byte_info['slidervalue'] = byte_info['slider'].GetValue()
        self.update_values()

    def valueToSlider(self, value):
        return (value) * float(RANGE) / (255)
        
    def sliderToValue(self, slider):
        pctvalue = slider / float(RANGE)
        return (255) * pctvalue
    
    def publish_values(self):
        pass
	message = String()
	min_vals = '%d,%d,%d' % (
	    self.color_values['h_min']['value'],
	    self.color_values['s_min']['value'],
	    self.color_values['v_min']['value'],
	)
	max_vals = '%d,%d,%d' % (
	    self.color_values['h_max']['value'],
	    self.color_values['s_max']['value'],
	    self.color_values['v_max']['value'],
	)
	message.data = ('%s|%s') % (min_vals, max_vals)
	self.color_publisher.publish(message)


app = wx.App()
gui = CalibrateGui("Calibrate Color GUI")
gui.Show()
app.MainLoop()
