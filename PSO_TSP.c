/* Libraries needed */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

/* Upper and lower limit constraints - here is is the map size */
#define LOWER	0
#define UPPER	200

/* Dimension of decision variable vector - number of cities */
#define Nvariables 15

/* Random number between [0, 1] */
#define Rand() ((double)rand()/RAND_MAX)

/* Particles structure */
typedef struct {
	int *x;
	double *v;
	double f;
	double pbest;
	int *x_star;
} ParticleRec, *Particle;

/*
	PSO Parameters
*/

/* Number of particles */
#define Nparticles 50
/* Maximum iteration count */
#define T_MAX 2000
/* Initial value of inertia coefficient: W_ 0 at t = 0, final value: W_T at t = T_MAX */
#define W_0		0.9
#define W_T		0.4
/* Maximum speed */
#define MAX_V		2.0
/* Cognitive parameters (c1) and social parameters (c2) */
#define c1		2.0
#define c2		2.0

/* Designation of minimization and maximization (here minimize) */
#define better(y1, y2) (y1 < y2)

/* City (node) structure */
typedef struct {
	double x;
	double y;
    int visited;
} City;

/* Redefine Nvariables to make code clearer */
#define NUM_CITIES Nvariables
City *cities;

/* Initialize memory for the cities */
void initializeCities()
{

    cities = malloc(NUM_CITIES * sizeof(City));
}

/* Place cities randomly on the map */
void populateCities()
{

    int i;

    for (i = 0; i < NUM_CITIES; i++)
    {
        cities[i].x = LOWER + (UPPER - LOWER)*Rand();
        cities[i].y = LOWER + (UPPER - LOWER)*Rand();
        printf("City [%d] position: x: %5.2f y: %5.2f\n", i, cities[i].x, cities[i].y);
    }
}

/* Cost of walking the cities */
void EvaluateParticle(Particle P)
{

	int i;

    P->f = 0.0;

    // Sum the costs of the paths
    for (i = 0; i < NUM_CITIES - 1; i++)
    {
        P->f += sqrt(pow(cities[P->x[i]].x - cities[P->x[i + 1]].x, 2) +
            pow(cities[P->x[i]].y - cities[P->x[i + 1]].y, 2));
    }

    // Return to the initial city
    P->f += sqrt(pow(cities[P->x[NUM_CITIES - 1]].x - cities[P->x[0]].x, 2) +
        pow(cities[P->x[NUM_CITIES - 1]].y - cities[P->x[0]].y, 2));
}

/* Best particle Update */
void UpdateBest(Particle P)
{
	int j;

	for(j = 0; j < Nvariables; j++)
    {

        P->x_star[j] = P->x[j];
    }
	P->pbest = P->f;
}

/* Initialization and evaluation of individuals */
/* Return index of best individual */
int Initialize(Particle P, int n)
{
	int i, j;
	int G;		/* Index of best particle */

	G = 0;
    int r;
    int tmp;

	for (i = 0; i < n; i++)
    {

        /* Shuffle the cities to avoid repetition */
        for (j = 0; j < Nvariables; j++)
        {
            P[i].x[j] = j;
        }

        for (j = Nvariables - 1; j > 0; j--) {

            r = (int) Nvariables * Rand();
            tmp = P[i].x[j];
            P[i].x[j] = P[i].x[r];
            P[i].x[r] = tmp;
            P[i].v[j] = 0.0;
        }

		EvaluateParticle(&P[i]);	/* Evaluation of individual */
		UpdateBest(&P[i]);
		if (better(P[i].f, P[G].f))
        {
            G = i;
        }
	}
	return G;
}

/* Macros and functions that dynamically assign arbitrary data types */
#define New(type, n, msg) (type *)NewCell(sizeof(type), n, msg)

void *NewCell(int size, int n, char *msg)
{
	void *new;

	if((new = malloc(size*n)) == NULL)
    {
		fprintf(stderr, "Cannot allocate memory for %d %s\n", n, msg);
		exit(1);
	}

	return new;
}

/* n Functions for dynamically allocating particles */
Particle NewParticles(int n)
{
	int i;
	Particle P;

	P = New(ParticleRec, n, "particles");
	for(i = 0; i < n; i++) {
		P[i].x = New(int, Nvariables, "x");
		P[i].v = New(double, Nvariables, "v");
		P[i].x_star = New(int, Nvariables, "x*");
	}
	return P;
}

/* Function that outputs particle (best information) */
void Print(Particle P)
{
	int j;

    printf("Best path: ");
	for (j = 0; j < Nvariables; j++)
    {
		printf("%d ", P->x_star[j]);
    }

	printf("\nMinimum path = %g\n", P->pbest);
}

/* Simple Particle Swarm Optimization */
int SPSO(Particle P)
{
	int t, i, j;
	int G;
	double w;
    int tmp;
    int newPos;

	/* Initialize the particles (place the cities) */
    G = Initialize(P, Nparticles);

	w = W_0;
	for (t = 1; t <= T_MAX; t++)
    {
		for (i = 0; i < Nparticles; i++)
        {
			for (j = 0; j < Nvariables; j++)
            {
                P[i].v[j] = w*P[i].v[j]
						+ c1*Rand()*(P[i].x_star[j] - P[i].x[j])
						+ c2*Rand()*(P[G].x_star[j] - P[i].x[j]);
				if (P[i].v[j] < -MAX_V)
					P[i].v[j] =- MAX_V;
				else if (P[i].v[j] > MAX_V)
					P[i].v[j] = MAX_V;
                newPos = P[i].x[j] + P[i].v[j];
                /* Test for boundaries */
                if (newPos < 0) newPos = 0;
                if (newPos > Nvariables - 1) newPos = Nvariables - 1;
                /* Shuffle two positions */
                tmp = P[i].x[j];
                P[i].x[j] = P[i].x[newPos];
                P[i].x[newPos] = tmp;
            }
            EvaluateParticle(&P[i]);
            /* Update better costs */
			if (better(P[i].f, P[i].pbest))
            {
				if (better(P[i].f, P[G].pbest))
                {
                    G = i;
                }
				UpdateBest(&P[i]);
			}
		}
		w -= (W_0 - W_T)/T_MAX;
	}
	return G;
}


int main(void)
{

	Particle P;
	int G;
	P = NewParticles(Nparticles);
    time_t t;
   	srand((unsigned) time(&t));
	initializeCities();
    populateCities();

	/* Perform the PSO algorithm */
    G = SPSO(P);

    /* Print the solution*/
	Print(&P[G]);

	return 0;
}
