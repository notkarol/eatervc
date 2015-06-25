#!/usr/bin/env python3
from subprocess import Popen, PIPE
import sqlite3
import pickle
import numpy
import sys
import os
#import matplotlib as mpl
#mpl.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import scipy.stats
import crossmodal

def pv_plot(ax, results, names, i, plot_name, title, ylabel, xlim=[0, 3], ylim=[0.0, 1.0], fontsize=12):
    datas = [result[i] for result in results]
    sig = [0 for result in results]
    
    for j in range(len(results)):
        if j == 1:
            sig[j] = 2
        t, prob = scipy.stats.ttest_ind(datas[1], datas[j])
        note = ''
        if prob < 0.05:
            note = '*' 
        if prob < 0.01:
            note = '**' 
        if prob < 0.001:
            note = '***' 
        sig[j] = 1 if prob < 0.05 else sig[j]
        #print('%s %s %s %6.3f %6.3f %2i prob %7.6f %s' %
        #      (plot_name, title, names[j], numpy.mean(datas[j]), numpy.min(datas[j]),
        #       numpy.sum(datas[j] < 0.25), prob, note))
        print('%s %s %s %6.3f prob %7.6f %s' %
              (plot_name, title, names[j], numpy.mean(datas[j]), prob, note))

    print()


    meanlineprops = dict(linestyle='-', linewidth=2.5, color='k')
    bp = ax.boxplot(datas,  meanprops=meanlineprops, meanline=True, showmeans=True)#, meanline=True)
    #ax.bar(numpy.arange(len(datas)), numpy.mean(datas, axis=1), color='blue')
    #ax.bar(numpy.arange(len(datas)), numpy.min(datas, axis=1), color='red')
    plt.setp(bp['boxes'], color='black')
    plt.setp(bp['whiskers'], color='black')
    plt.setp(bp['fliers'], color='red', marker='+')
    if i == 2:
        ax.plot([0, len(datas) + 1], [3.16667, 3.16667], color='y', alpha=0.3)
    if i == 3:
        ax.plot([0, len(datas) + 1], [3.12607, 3.12607], color='y', alpha=0.3)

    # Color in the boxes
    boxColors = ['w', 'mediumseagreen', 'lightsteelblue']
    numBoxes = len(datas)
    medians = numpy.arange(numBoxes)
    for i in range(numBoxes):
        box = bp['boxes'][i]
        boxX = []
        boxY = []
        for j in range(5):
            boxX.append(box.get_xdata()[j])
            boxY.append(box.get_ydata()[j])        
        boxCoords = numpy.array(list(zip(boxX,boxY)))
        boxPolygon = Polygon(boxCoords, facecolor=boxColors[sig[i]])
        ax.add_patch(boxPolygon)

    # rest of the plot
    ax.set_axisbelow(True)    
    ax.set_xticklabels(names)#, rotation=90)
    ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)
    ax.set_xlabel('Modality', fontsize=fontsize)
    ax.set_ylabel(ylabel, fontsize=fontsize)
    ax.set_title(title, fontsize=fontsize)
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    #ax.set_aspect(1.5)
    #plt.grid()
    #ax.spines['top'].set_visible(False)
    #ax.spines['right'].set_visible(False)
    #ax.spines['bottom'].set_visible(False)
    #ax.spines['left'].set_visible(False)

def main():

    p = 'w000_aP5_b55r_sW_v00_f0'
    #v = 'w000_aD5_b55r_sW_v00_f0'
    v = 'w000_aV5_b55r_sW_v00_f0'
    p_test_path = "%s/data/%s/%s" % (crossmodal.crossmodal_path(), sys.argv[1], p)
    v_test_path = "%s/data/%s/%s" % (crossmodal.crossmodal_path(), sys.argv[1], v)
    v_test_paths = os.listdir(v_test_path)

    conds = ['vA4', 'vA6', 'vA8',
             'vH4', 'vH6', 'vH8',
             'vV4', 'vV6', 'vV8']
    for path in sorted(os.listdir(p_test_path)):
        if path[-2:] != '.p': continue
        paths = ["%s/%s" % (p_test_path, path)]
        
    
    for path in sorted(os.listdir(p_test_path)):
        if path[-2:] != '.p': continue
        paths = ["%s/%s" % (p_test_path, path)]
        p_w, p_a, p_b, p_s, p_v, p_f = path[:-2].split('_')
        #if p_w != 'w040' or p_v[2] != '4': continue
        if p_w != 'w060' or p_v[2] != '6': continue
        #if p_w != 'w080' or p_v[2] != '8': continue
        #for letter in 'DXSC':
        for letter in 'VXSC':
            for vpath in v_test_paths:
                if vpath[-2:] != '.p': continue
                v_w, v_a, v_b, v_s, v_v, v_f = vpath[:-2].split('_')
                if p_w == v_w and p_b == v_b and p_s == v_s and p_v == v_v and p_f == v_f and v_a[1] == letter:
                    paths.append("%s/%s" % (v_test_path, vpath))
                    #print(paths)
        create_pv_plot('%s_%s_%s' % (sys.argv[1], p_w, p_v), paths, int(sys.argv[2]))
    return 0

def calc_results(results, runs, h, goals):
    for pos, run_id in enumerate(sorted(runs)):
        #print(pos, run_id)
        trial_fits = runs[run_id][0][1]
        test_fits = runs[run_id][0][3]
        test_outs = runs[run_id][0][4]

        # test set error
        mean_fit = numpy.mean(test_fits)
        results[0, pos] = mean_fit

        # Motion
        if len(results) < 2:
            continue
        motion = 0.0
        for test_out in test_outs:
            d = numpy.abs(test_out[1:, :2] - test_out[:-1, :2])
            motion += numpy.sum(numpy.sqrt(numpy.power(d[:, 0], 2) + numpy.power(d[:, 1], 2)))
        results[1, pos] = motion / len(test_outs)

        # intra inter
        if len(results) < 3: continue
        xzs = numpy.zeros((len(test_outs), 2))
        for pos_out, test_out in enumerate(test_outs):
            xzs[pos_out, :] = test_out[-1, 0 : 2]
            #xzs[pos_out, :] = test_out[0, 0 : 2]
        intra, inter = crossmodal.intra_inter(xzs, goals)
        results[2, pos] = intra
        results[3, pos] = inter
        results[4, pos] = numpy.abs(inter - intra)
        
        # intra inter hidden
        #if len(results) < 5: continue
        #hiddens = numpy.zeros((len(test_outs), h))
        #for pos_out, test_out in enumerate(test_outs):
        #    hiddens[pos_out, :] = test_out[-1, 7 : 7 + h]
        #intra, inter = crossmodal.intra_inter(hiddens, goals)
        #results[4, pos] = intra
        #results[5, pos] = inter
    
def create_pv_plot(plot_name, paths, num_plots=6):
    results = []
    names = []
    for ppath in paths:
        name = os.path.basename(ppath)[:-2]
        path = os.path.dirname(ppath)
        condids, goals = crossmodal.fetch_condids_goals(path)

        try:
            with open(ppath, 'rb') as f:
                runs = pickle.load(f)
        except FileNotFoundError:
            print('File not found:', ppath)
            return None
        
        h = int(name[7])

        print(ppath, len(runs))
        results.append(numpy.zeros((num_plots, len(runs))))
        calc_results(results[-1], runs, h, goals)
        names.append(name[6])

    cmap=plt.get_cmap('copper')
    if num_plots == 6:
        fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(nrows=3, ncols=2)
        fig.set_figheight(8)
        fig.set_figwidth(8)
    if num_plots == 5:
        fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(nrows=3, ncols=2)
        fig.set_figheight(8)
        fig.set_figwidth(6)
        ax6.axis('off')
    if num_plots == 4:
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(nrows=2, ncols=2)
        fig.set_figheight(6)
        fig.set_figwidth(8)
    if num_plots == 2:
        fig, ((ax1, ax2)) = plt.subplots(nrows=1, ncols=2)
        fig.set_figheight(4)
        fig.set_figwidth(8)
    if num_plots == 1:
        fig, (ax1) = plt.subplots(nrows=1, ncols=1)
        fig.set_figheight(8)
        fig.set_figwidth(8)
    
    pv_plot(ax1, results, names, 0, plot_name, 'Test Error', 'Test Error',
            [0, len(results) + 1], [0.0, 0.65])
    if num_plots > 1:
        pv_plot(ax2, results, names, 1, plot_name, 'Motion', 'Units',
                [0, len(results) + 1], [0.0, 20.0])
    if num_plots > 3:
        pv_plot(ax3, results, names, 2, plot_name, 'Intra-Category Distance\nof Movement', 'Distance',
                [0, len(results) + 1], [0.0, 5.])
        pv_plot(ax4, results, names, 3, plot_name, 'Inter-Category Distance\nof Movement', 'Distance',
                [0, len(results) + 1], [0.0, 5.])
    if num_plots == 5:
        pv_plot(ax5, results, names, 4, plot_name, 'Intra-Inter Category\nDistance of Movement', 'Distance',
                [0, len(results) + 1], [0.0, 2.])
    if num_plots == 6:
        pv_plot(ax5, results, names, 4, plot_name, 'Intra-Category of Hidden Layer', '',
                [0, len(results) + 1], [0.0, 0.5])
        pv_plot(ax6, results, names, 5, plot_name, 'Inter-Category of Hidden Layer', '',
                [0, len(results) + 1], [0.0, 0.5])
    fig.tight_layout()
    plt.savefig('pv_%s.png' % (plot_name), bbox_inches='tight', dpi=200)
    plt.savefig('pv_%s.eps' % (plot_name), bbox_inches='tight', dpi=200)
    plt.close()
    #plt.show()
                
if __name__ == "__main__":
    sys.exit(main())
