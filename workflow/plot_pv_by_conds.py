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

def pv_plot(ax, results, names, plot_num, plot_name, title, ylabel, xlim=[0, 3], ylim=[0.0, 1.0], fontsize=12):
    datas = [result[plot_num] for result in results]
    means = [numpy.mean(result[plot_num]) for result in results]
    stds = [numpy.std(result[plot_num]) for result in results]
    sig = [0 for result in results]

    for j in range(len(results)):
        if names[j][4] != 'V': continue
        sig[j] = 2
        for k in range(len(results)):
            if names[k][4] == 'V': continue
            if names[j][0] != names[k][0]: continue
            t, prob = scipy.stats.ttest_ind(datas[j], datas[k])
            note = '*' if prob < 0.05 else ''
            sig[k] = 1 if prob < 0.05 else 0
            print('%s %s %s %6.3f prob %5.3f %s' %
                  (plot_name, title, names[k], numpy.mean(datas[k]), prob, note))
    print()

    new_names = [name + ('*' if sig[j] else '') for j, name in enumerate(names)]
    
    bp = ax.boxplot(datas)
    plt.setp(bp['boxes'], color='black')
    plt.setp(bp['whiskers'], color='black')
    plt.setp(bp['fliers'], color='red', marker='+')
    ax.set_xticklabels(new_names, rotation=90)
    ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)

    # Color in the boxes
    boxColors = ['w', 'k', 'royalblue']
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

        # Now draw the median lines back over what we just filled in
        med = bp['medians'][i]
        medianX = []
        medianY = []
        for j in range(2):
            medianX.append(med.get_xdata()[j])
            medianY.append(med.get_ydata()[j])
            plt.plot(medianX, medianY, 'k')
            medians[i] = medianY[0]
            # Finally, overplot the sample averages, with horizontal alignment
            # in the center of each box
            plt.plot([numpy.mean(med.get_xdata())], [numpy.mean(datas[i])],
                     color='w', marker='*', markeredgecolor='k')
    ax.set_axisbelow(True)
    ax.set_xlabel('Modality', fontsize=fontsize)
    ax.set_ylabel(ylabel, fontsize=fontsize)
    ax.set_title(title, fontsize=fontsize)
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)

def main():

    p = 'w000_aP5_b55r_sW_v00_f0'
    v = 'w000_aV5_b55r_sW_v00_f0'
    p_test_path = "%s/data/gecco5/%s" % (crossmodal.crossmodal_path(), p)
    v_test_path = "%s/data/gecco5/%s" % (crossmodal.crossmodal_path(), v)

    conds = ['vA4', 'vA6', 'vA8',
             'vH4', 'vH6', 'vH8',
        if len(results) < 3: continue
        hiddens = numpy.zeros((len(test_outs), h))
        for pos_out, test_out in enumerate(test_outs):
            hiddens[pos_out, :] = test_out[0, 7 : 7 + h]
        intra, inter = crossmodal.intra_inter(hiddens, goals)
        results[2, pos] = intra
        results[3, pos] = inter

        # intra inter
        if len(results) < 5: continue
        xzs = numpy.zeros((len(test_outs), 2))
        for pos_out, test_out in enumerate(test_outs):
            xzs[pos_out, :] = test_out[-1, 0 : 2]
        intra, inter = crossmodal.intra_inter(xzs, goals)
        results[4, pos] = intra
        results[5, pos] = inter

    
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

        results.append(numpy.zeros((num_plots, len(runs))))
        calc_results(results[-1], runs, h, goals)
        names.append('%sk %s (%i)' % (name[2:4], name[6], len(runs)))

    cmap=plt.get_cmap('copper')
    if num_plots == 6:
        fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(nrows=3, ncols=2)
        fig.set_figheight(12)
    if num_plots == 4:
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(nrows=2, ncols=2)
        fig.set_figheight(9)
    if num_plots == 2:
        fig, ((ax1, ax2)) = plt.subplots(nrows=1, ncols=2)
        fig.set_figheight(6)
    if num_plots == 1:
        fig, (ax1) = plt.subplots(nrows=1, ncols=1)
        fig.set_figheight(12)
    fig.set_figwidth(12)
    
    pv_plot(ax1, results, names, 0, plot_name, 'Test Error', 'Test Error',
            [0, len(results) + 1], [0.0, 0.65])
    if num_plots > 1:
        pv_plot(ax2, results, names, 1, plot_name, 'Motion', 'Units',
                [0, len(results) + 1], [0.0, 20.0])
    if num_plots > 3:
        pv_plot(ax3, results, names, 2, plot_name, 'Mean Intra Category Distance of Hidden Layer', 'Distance',
                [0, len(results) + 1], [0.0, 0.5])
        pv_plot(ax4, results, names, 3, plot_name, 'Mean Inter Category Distance of Hidden Layer', 'Distance',
                [0, len(results) + 1], [0.0, 0.5])
    if num_plots > 5:
        pv_plot(ax5, results, names, 5, plot_name, 'Mean Intra Category Distance of Movement', 'Distance',
                [0, len(results) + 1], [0.0, 3.])
        pv_plot(ax6, results, names, 5, plot_name, 'Mean Inter Category Distance of Movement', 'Distance',
                [0, len(results) + 1], [0.0, 3.])
    plt.savefig('pv_%s.png' % (plot_name), bbox_inches='tight', dpi=200)
    #plt.savefig('pv_%s.eps' % (plot_name), bbox_inches='tight', dpi=200)
    plt.close()
    #plt.show()
                
if __name__ == "__main__":
    sys.exit(main())
