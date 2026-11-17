#include "LKH.h"
#include "Genetic.h"
#include "gpx.h"
#include "Sequence.h"

/* Global variables defined in LKH.h */
int AscentCandidates;
int Asymmetric;
int BackboneTrials;
int Backtracking;
GainType BestCost;
GainType BestPenalty;
int *BestTour;
GainType BetterCost;
GainType BetterPenalty;
int *BetterTour;
int BWTSP_B;
int BWTSP_Q;
int BWTSP_L;
int CacheMask;
int *CacheVal;
int *CacheSig;
int CandidateFiles;
int EdgeFiles;
int *CostMatrix;
GainType CurrentGain;
GainType CurrentPenalty;
int DemandDimension;
Node *Depot;
int Dimension;
int DimensionSaved;
int Dim;
double DistanceLimit;
double Excess;
int ExtraCandidates;
Node *FirstActive, *LastActive;
Constraint *FirstConstraint;
Node *FirstNode;
Segment *FirstSegment;
SSegment *FirstSSegment;
int Gain23Used;
int GainCriterionUsed;
double GridSize;
int GroupSize;
int SGroupSize;
int Groups;
int SGroups;
unsigned Hash;
Node **Heap;
HashTable *HTable;
int InitialPeriod;
int InitialStepSize;
double InitialTourFraction;
int Kicks;
int KickType;
char *LastLine;
double LowerBound;
int M;
int MaxBreadth;
int MaxCandidates;
int MaxMatrixDimension;
int MaxSwaps;
int MaxTrials;
int MergeTourFiles;
int MoveType;
int MoveTypeSpecial;
Node *NodeSet;
int Norm;
int NonsequentialMoveType;
GainType Optimum;
int PatchingA;
int PatchingC;
GainType PenaltyGain;
int Precision;
int PredSucCostAvailable;
int POPMUSIC_InitialTour;
int POPMUSIC_MaxNeighbors;
int POPMUSIC_SampleSize;
int POPMUSIC_Solutions;
int POPMUSIC_Trials;
unsigned *Rand;
int Recombination;
int RestrictedSearch;
short Reversed;
int Run;
int Runs;
int Scale;
double ServiceTime;
int Serial;
unsigned Seed;
int StopAtOptimum;
int Subgradient;
int SubproblemSize;
int SubsequentMoveType;
int SubsequentMoveTypeSpecial;
int SubsequentPatching;
SwapRecord *SwapStack;
int Swaps;
int OldSwaps;
double TimeLimit;
int TotalDemand;
int TraceLevel;
int Trial;
GainType TSPTW_CurrentMakespanCost;
int TSPTW_Makespan;

/* The following variables are read by the functions ReadParameters and
   ReadProblem: */

char *ParameterFileName, *ProblemFileName, *PiFileName,
    *TourFileName, *OutputTourFileName, *InputTourFileName,
    **CandidateFileName, **EdgeFileName, *InitialTourFileName,
    *SubproblemTourFileName, **MergeTourFileName,
    *MTSPSolutionFileName, *SINTEFSolutionFileName;
char *Name, *Type, *EdgeWeightType, *EdgeWeightFormat,
    *EdgeDataFormat, *NodeCoordType, *DisplayDataType;
int CandidateSetSymmetric, CandidateSetType, Capacity,
    CoordType, DelaunayPartitioning, DelaunayPure,
    ExternalSalesmen,
    ExtraCandidateSetSymmetric, ExtraCandidateSetType,
    InitialTourAlgorithm,
    KarpPartitioning, KCenterPartitioning, KMeansPartitioning,
    MTSPDepot, MTSPMinSize, MTSPMaxSize, MTSPObjective,
    MoorePartitioning,
    PatchingAExtended, PatchingARestricted,
    PatchingCExtended, PatchingCRestricted,
    ProblemType, RiskThreshold,
    RohePartitioning, Salesmen, SierpinskiPartitioning,
    SubproblemBorders, SubproblemsCompressed, WeightType, WeightFormat;

FILE *ParameterFile, *ProblemFile, *PiFile, *InputTourFile,
    *TourFile, *InitialTourFile, *SubproblemTourFile, **MergeTourFile;
CostFunction Distance, D, C, c, OldDistance;
MoveFunction BestMove, BacktrackMove, BestSubsequentMove;
PenaltyFunction Penalty;
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