/*
 * This file defines global variables that are declared as extern in the header files.
 * This resolves the multiple definition errors that occur when headers are included
 * in multiple source files.
 */

#include "INCLUDE/LKH.h"
#include "INCLUDE/Sequence.h"
#include "INCLUDE/Genetic.h"

/* Variables from LKH.h */
MergeTourFunction MergeWithTour;

/* Variables from Sequence.h */
Node **t;
Node **T;
Node **tSaved;
int *p;
int *q;
int *incl;
int *cycle;
GainType *G;
int K;

/* Variables from Genetic.h */
int MaxPopulationSize;
int PopulationSize;
CrossoverFunction Crossover;
int **Population;
GainType *PenaltyFitness;
GainType *Fitness;

/* Variables from gpx.h */
int n_cities, n_cand;
int n_partitions_size2, n_partitions_before_fusion,
    n_partitions_after_fusion1, n_partitions_after_fusion2,
    n_partitions_after_fusion3;
int n_partitions_after_fusion4, n_partitions_after_fusion5,
    n_partitions_after_fusionB;
Node **Map2Node;