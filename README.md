# eatervc

This repository contains the code and data for replicating the paper entited "An Embodied Approach for Evolving Robust 
Visual Classifiers"

Additionally, the talk given at GECCO 2015 can be found in the following link. Please be sure to enable captions as the audio can be hard to hear. https://www.youtube.com/watch?v=YKW8ZJPxBlQ

All images and LaTeX files used to generate the PDF are included in the latex folder of this repository.

Also, I've written a simple step-by-step guide for reproducing this work in Bullet: https://www.reddit.com/r/ludobots/comments/3jy8x9/project_reproducing_an_embodied_approach_for/

Non-Data Figures:

Figure 1:

- The most basic way to regenerate this figure is to compile and run "gevaluator" from a directory with an initialized experiment in data/EXPERIMENT/CONDITIONS/. The executable allows command line arguments of the starting position followed by up to 75 synapse weights. 

Figure 2:

- This figure was generated using the TikZ latex package and can be found in the latex/gecco.tex file.

Figure 3:

- This figure was generated using the "workflow/positions.py" python script, using the settings of "settings/gecco.py" to produce at Matplotlib plot.

Figures 4 and 5:

- These figures were generated using the "workflow/scaffold.py" python script to produce at Matplotlib plot.



Data Figures:

Table 1:

- These data points can be found within each of the file in the "results" folder. Using Python and the pickle library you can load these test results. The results are stored in a python dict, where the first fifty (numerically) entries contain the results presented in the paper. I've since run more results and included them. Each run's tests store the run's test mean as the fourth entry of the first array. Finding the means and then running a tw-tailed T-Test on these values will provide the values and p-values.

Figures 6 and 7:

- These charts were created using the "workflow/pv.py" python script. As in re-generating "Table 1" data, the pickled test results can be plotted. For each run, the first array item in each dict entries contains the test reults in a list. Fromw ithin this list the overall test fitness can be found in the fourth array entry. The fifth entry contains a numpy array of data generated within the runs. The first two columns store the X and Z positions of the cylinders and can be used to calculate the net displacement of the blocks. Similarly, for intra-inter distances, can be calculated using these displacement values. The first 78 test results are of the smaller objects and the remaining 78 test results are of the larger objects.



