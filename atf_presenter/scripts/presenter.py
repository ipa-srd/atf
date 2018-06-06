#!/usr/bin/env python
import numpy
import rospy
import yaml
import copy
import os
import sys
import optparse
#import matplotlib.pyplot as plt
from matplotlib import rcParams
rcParams.update({'figure.autolayout': True})
import collections
from pylab import *


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


    def extract_loc_from_yaml(self, num):
        self.numvals = {}
        self.numvals["trans"] = []
        self.numvals["rot"] = []
        #print "len: " + str(len(self.numvals))
        #print self.yaml_file
        for testblock, metrics in self.yaml_file.iteritems():
            #print ("testblock: ", testblock)
            for metric, data in metrics.iteritems():
                #print ("metric: ", metric)
                #print metric[1].find("localization")
                if not "localization" in metric:
                    continue
                self.metric.add(metric)
                for values in data:
                    for name, result in values.iteritems():
                        if name == 'data':
                            i = 0
                            j = 0
                            for data_name in result:
                                if (data_name == "max"):
                                    for key, vals in result[data_name].iteritems():
                                        if isinstance(vals, collections.Iterable):
                                            for data_point in vals:
                                                if key == "trans":
                                                    #print str(i) +": "+ str(data_point)
                                                    i = i + 1
                                                    self.numvals["trans"].append(data_point)
                                                    #print self.numvals[str(num)]["trans"]
                                                elif key == "rot":
                                                    #print str(j) + ": " + str(data_point)
                                                    j = j + 1
                                                    self.numvals["rot"].append(data_point)
                                        else:
                                            self.numvals[key] = vals
        return self.numvals

    def extract_cpu_from_yaml(self, num):
        for testblock, metrics in self.yaml_file.iteritems():
            #print ("testblock: ", testblock)
            for metric, data in metrics.iteritems():
                #print ("metric: ", metric)
                #print metric[1].find("localization")
                if not "cpu" in metric:
                    continue
                self.metric.add(metric)
                for values in data:
                    for name, result in values.iteritems():
                        if name == 'data':
                            for data_name in result:
                                if (data_name == "max"):
                                    return result[data_name]
        # plt.figure(1)
        # print self.numvals[str(num)].keys()
        # self.box_plt = plt.boxplot(
        #     self.numvals[str(num)]["trans"], positions=[int(num)], widths=0.6)
        # plt.plot([int(num) - 0.3, int(num), int(num) + 0.3],
        #          [self.numvals[str(num)]["avg_trans"], self.numvals[str(num)]["avg_trans"]
        #              , self.numvals[str(num)]["avg_trans"]], color='g')
        #
        # if self.min_trans[1] > self.numvals[str(num)]["avg_trans"]:
        #     self.min_trans = [str(num), self.numvals[str(num)]["avg_trans"]]
        # if self.min_med_trans[1] > self.box_plt['medians'][0].get_ydata()[0]:
        #     self.min_med_trans = [str(num), self.box_plt['medians'][0].get_ydata()[0]]
        # plt.figure(2)
        # self.box_plt = plt.boxplot(
        #     self.numvals[str(num)]["rot"], positions=[int(num)], widths=0.6)
        # plt.plot([int(num) - 0.3, int(num), int(num) + 0.3],
        #          [self.numvals[str(num)]["avg_rot"], self.numvals[str(num)]["avg_rot"]
        #              , self.numvals[str(num)]["avg_rot"]], color='g')
        #
        # if self.min_rot[1] > self.numvals[str(num)]["avg_rot"]:
        #     self.min_rot = [str(num), self.numvals[str(num)]["avg_rot"]]
        # if self.min_med_rot[1] > self.box_plt['medians'][0].get_ydata()[0]:
        #     self.min_med_rot = [str(num), self.box_plt['medians'][0].get_ydata()[0]]
        # self.pos = self.pos + 1

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

    #print p.testlist

    robot_configs = {}
    for test in p.testlist:
        # extract robot num
        #print test
        for key in test:
            #print key
            #print test[key]
            pos = key.find('_r')
            #print pos
            end = key.find('_', pos + 1)
            #print end
            robot_num = key[pos + 2 : end]
            #print "robot: " + str(robot_num)
            #extract_config
            config = test[key]["robot"]
            robot_configs.update({robot_num: config})

    #print robot_configs

    robot_results = {}
    tick_counter = 0
    for file in filelist:
        if "merged" in str(file):
            p.import_yaml(Path+file)
            filename = file.replace('.yaml', '')
            pos = filename.find('_r')
            #print pos
            end = filename.find('_', pos + 1)
            #print end
            robot_num = filename[pos + 2 : end]
            #print "robot: " + str(robot_num)
            robot_vals = p.extract_loc_from_yaml(robot_num)
            robot_cpu = p.extract_cpu_from_yaml(robot_num)
            #tick_counter = tick_counter + 1
            robot_results.update({robot_num: [robot_vals, robot_cpu]})
    config_results = {}
    for key in robot_configs:
        conf = robot_configs[key]
        res = robot_results[key]
        config_results.update({conf:res})

    # for key in config_results:
    #     print key

    #print config_results

    mcl_res_dict = {}
    mcl_map_res_dict = {}
    rbpf_res_dict = {}
    for key in config_results:
        res = float(key.split("_",1)[0])
        pos_error = config_results[key][0]["trans"]
        cpu = config_results[key][1]
        key
        if "false_false" in key:
            mcl_res_dict.update({res: [pos_error, cpu]})
            #print "mcl: res:", res
        elif "true_true" in key:
            mcl_map_res_dict.update({res: [pos_error, cpu]})
            #print "map: res:", res
        elif "true_false" in key:
            rbpf_res_dict.update({res: [pos_error, cpu]})
            #print "rb: res:", res

    res_range = []
    for key in mcl_res_dict:
        res_range.append(key)
    res_range.sort()

    mcl_res_list = []
    mcl_mean_list = []
    mcl_stddev_list = []
    mcl_cpu_list = []
    mcl_map_res_list = []
    mcl_map_mean_list = []
    mcl_map_stddev_list = []
    mcl_map_cpu_list = []
    rbpf_res_list = []
    rbpf_mean_list = []
    rbpf_stddev_list = []
    rbpf_cpu_list = []
    for res in res_range:
        vals = mcl_res_dict[res][0]
        mcl_res_list.append(vals)
        mean = numpy.mean(vals)
        std_dev = numpy.std(vals)
        mcl_mean_list.append(mean)
        mcl_stddev_list.append(std_dev)
        cpu = mcl_res_dict[res][1]
        mcl_cpu_list.append(cpu)
        vals = mcl_map_res_dict[res][0]
        mcl_map_res_list.append(vals)
        mean = numpy.mean(vals)
        std_dev = numpy.std(vals)
        mcl_map_mean_list.append(mean)
        mcl_map_stddev_list.append(std_dev)
        cpu = mcl_map_res_dict[res][1]
        mcl_map_cpu_list.append(cpu)
        vals = rbpf_res_dict[res][0]
        rbpf_res_list.append(vals)
        mean = numpy.mean(vals)
        std_dev = numpy.std(vals)
        rbpf_mean_list.append(mean)
        rbpf_stddev_list.append(std_dev)
        cpu = rbpf_res_dict[res][1]
        rbpf_cpu_list.append(cpu)

# plotting
line_style = ['-o', '-.o', ':o', '-o']
line_color = ['b', 'g', 'r', 'darkorange']
font = {'family' : 'sans-serif', #['serif', 'sans-serif', 'cursive', 'fantasy', 'monospace']
        'weight' : 'normal', # ['light', 'normal', 'medium', 'semibold', 'bold', 'heavy', 'black']
        'size'   : 22}
matplotlib.rc('font', **font)
labels = ["MCL", "MCL-MAP-Mapping", "RBPF"]


fig = plt.figure()
ax1 = fig.add_subplot(2,1,1)
ax2 = fig.add_subplot(2,1,2)
plt.title("Static environment")

ax1.errorbar(res_range, mcl_mean_list, yerr=mcl_stddev_list, fmt=line_style[0], color=line_color[0], label=labels[0], linewidth=2.0)
ax1.errorbar(res_range, mcl_map_mean_list, yerr=mcl_map_stddev_list, fmt=line_style[1], color=line_color[1], label=labels[1], linewidth=2.0)
ax1.errorbar(res_range, rbpf_mean_list, yerr=rbpf_stddev_list, fmt=line_style[2], color=line_color[2], label=labels[2], linewidth=2.0)

ax1.set_xlabel('Grid cell size [m]')
ax1.set_ylabel('Mean position error [m]')
leg = ax1.legend(mode=None, shadow=False, fancybox=True)
leg.get_frame().set_alpha(0.5)
axes = plt.gca()
ax1.set_ylim([0,1])

width = 1
ax2.errorbar(res_range, mcl_cpu_list, yerr=0, fmt=line_style[0], color=line_color[0], label=labels[0], linewidth=2.0)
ax2.errorbar(res_range, mcl_map_cpu_list, yerr=0, fmt=line_style[1], color=line_color[1], label=labels[1], linewidth=2.0)
ax2.errorbar(res_range, rbpf_cpu_list, yerr=0, fmt=line_style[2], color=line_color[2], label=labels[2], linewidth=2.0)
ax1.set_ylabel('CPU Load')

plt.show()
#axes.xmargin=0.2
#axes.ymargin=0.2

    # print "min mean trans: " + str(p.min_trans)
    # print "min mean rot: " + str(p.min_rot)
    # print "min median trans: " + str(p.min_med_trans)
    # print "min median rot: " + str(p.min_med_rot)
    # ticks = []
    # ticks.append(-1)
    # for i in xrange(tick_counter):
    #     ticks.append(i)
    # ticks.append(tick_counter)
    # plt.figure(1)
    # plt.xticks(ticks, ticks)
    # ax = plt.figure(1).add_subplot(111)
    # ax.set_xlabel('Robot Number')
    # ax.set_ylabel('delta trans in m')
    # plt.figure(2)
    # plt.xticks(ticks, ticks)
    # ax = plt.figure(2).add_subplot(111)
    # ax.set_xlabel('Robot Number')
    # ax.set_ylabel('delta rot in degree')
    # plt.show()
