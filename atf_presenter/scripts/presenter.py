#!/usr/bin/env python
import numpy
import rospy
import yaml
import copy
import os
import sys
import optparse
import matplotlib.pyplot as plt
import collections


class presenter:

    def __init__(self):
        self.filepath = "~"
        self.yaml_file = {}
        self.testblock = []
        self.metric = set()
        self.data = {}
        self.tests = {}
        self.testnames = []
        self.testlist = {}
        self.pos = 0
        self.numvals = {}
        self.min_trans = ['', sys.float_info.max]
        self.min_rot = ['', sys.float_info.max]
        self.min_med_trans = ['', sys.float_info.max]
        self.min_med_rot = ['', sys.float_info.max]



    def import_yaml(self, file):
        with open(file, 'r') as stream:
            print "import file", file
            try:
                #print(yaml.load(stream))
                self.yaml_file = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        for testblock, metrics in self.yaml_file.iteritems():
            for metric, values in metrics.iteritems():
                self.data.update({metric : []})


    def extract_yaml(self, num):
        self.numvals[str(num)] = {}
        self.numvals[str(num)]["trans"] = []
        self.numvals[str(num)]["rot"] = []
        print "len: " + str(len(self.numvals))
        for testblock, metrics in self.yaml_file.iteritems():
            #print ("testblock: ", testblock)
            for metric, data in metrics.iteritems():
                #print ("metric: ", metric)
                self.metric.add(metric)
                for values in data:
                    for name, result in values.iteritems():
                        if name == 'data':
                            i = 0
                            j = 0
                            for data_name in result:
                                if isinstance(result[data_name], collections.Iterable):
                                    for data_point in result[data_name]:
                                        if data_name == "trans":
                                            #print str(i) +": "+ str(data_point)
                                            i = i + 1
                                            self.numvals[str(num)]["trans"].append(data_point)
                                        elif data_name == "rot":
                                            #print str(j) + ": " + str(data_point)
                                            j = j + 1
                                            self.numvals[str(num)]["rot"].append(data_point)
                                else:
                                    self.numvals[str(num)][data_name] = result[data_name]
        plt.figure(1)
        self.box_plt = plt.boxplot(
            self.numvals[str(num)]["trans"], positions=[int(num)], widths=0.6)
        plt.plot([int(num) - 0.3, int(num), int(num) + 0.3],
                 [self.numvals[str(num)]["avg_trans"], self.numvals[str(num)]["avg_trans"]
                     , self.numvals[str(num)]["avg_trans"]], color='g')

        if self.min_trans[1] > self.numvals[str(num)]["avg_trans"]:
            self.min_trans = [str(num), self.numvals[str(num)]["avg_trans"]]
        if self.min_med_trans[1] > self.box_plt['medians'][0].get_ydata()[0]:
            self.min_med_trans = [str(num), self.box_plt['medians'][0].get_ydata()[0]]
        plt.figure(2)
        self.box_plt = plt.boxplot(
            self.numvals[str(num)]["rot"], positions=[int(num)], widths=0.6)
        plt.plot([int(num) - 0.3, int(num), int(num) + 0.3],
                 [self.numvals[str(num)]["avg_rot"], self.numvals[str(num)]["avg_rot"]
                     , self.numvals[str(num)]["avg_rot"]], color='g')

        if self.min_rot[1] > self.numvals[str(num)]["avg_rot"]:
            self.min_rot = [str(num), self.numvals[str(num)]["avg_rot"]]
        if self.min_med_rot[1] > self.box_plt['medians'][0].get_ydata()[0]:
            self.min_med_rot = [str(num), self.box_plt['medians'][0].get_ydata()[0]]
        self.pos = self.pos + 1

    def import_testnames(self, file):
        with open(file, 'r') as stream:
            self.testlist = yaml.load(stream)

if __name__ == '__main__':
    parser = optparse.OptionParser()
    parser.add_option('-s', '--single', dest='single', help='Print all plots in single windows', default=False, action="store_true")
    parser.add_option('-p', '--path', dest='path', help='Target Path', default="~")
    (options, args) = parser.parse_args()

    p = presenter()
    Path = options.path
    filelist = os.listdir(Path)
    p.import_testnames(Path.replace('yaml', 'json')+"test_list.json")

    tick_counter = 0
    for file in filelist:
        if "merged" in str(file):
            p.import_yaml(Path+file)
            filename = file.replace('.yaml', '')
            pos = filename.find('_r')
            print pos
            end = filename.find('_', pos + 1)
            print end
            robot_num = filename[pos + 2 : end]
            print "robot: " + str(robot_num)
            p.extract_yaml(robot_num)
            tick_counter = tick_counter + 1
    print "min mean trans: " + str(p.min_trans)
    print "min mean rot: " + str(p.min_rot)
    print "min median trans: " + str(p.min_med_trans)
    print "min median rot: " + str(p.min_med_rot)
    ticks = []
    ticks.append(-1)
    for i in xrange(tick_counter):
        ticks.append(i)
    ticks.append(tick_counter)
    plt.figure(1)
    plt.xticks(ticks, ticks)
    ax = plt.figure(1).add_subplot(111)
    ax.set_xlabel('Robot Number')
    ax.set_ylabel('delta trans in m')
    plt.figure(2)
    plt.xticks(ticks, ticks)
    ax = plt.figure(2).add_subplot(111)
    ax.set_xlabel('Robot Number')
    ax.set_ylabel('delta rot in degree')
    plt.show()