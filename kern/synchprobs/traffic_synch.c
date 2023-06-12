#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>

/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

/*
 * replace this with declarations of any synchronization and other variables you need here
 */
// static struct semaphore *intersectionSem;
static struct lock * traffic_lock;
static struct cv * traffic_cv[4];
static int cars_at_intersection = 0;
static int waiting_cars[4] = {0};
static Direction safe_to_go;


/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */
void
intersection_sync_init(void)
{
  /* replace this default implementation with your own implementation */
  /*
  intersectionSem = sem_create("intersectionSem",1);
  if (intersectionSem == NULL) {
    panic("could not create intersection semaphore");
  }
  return;
  */
  traffic_lock = lock_create("traffic_lock");
  for (int i = 0; i < 4; i++) {
    traffic_cv[i] = cv_create("cv_"+i);
  }
  if (traffic_lock == NULL) {
    panic("Traffic Lock creation failure");
  }
  return;
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
  /* replace this default implementation with your own implementation */
  /*
  KASSERT(intersectionSem != NULL);
  sem_destroy(intersectionSem);
  */
  KASSERT(traffic_lock != NULL);
  for (int i = 0; i < 4; i++) {
    cv_destroy(traffic_cv[i]);
  }
  lock_destroy(traffic_lock);
  return;
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void
intersection_before_entry(Direction origin, Direction destination) 
{
  /* replace this default implementation with your own implementation */
  // (void)origin;  /* avoid compiler complaint about unused parameter */
  (void)destination; /* avoid compiler complaint about unused parameter */
  // KASSERT(intersectionSem != NULL);
  // P(intersectionSem);
  lock_acquire(traffic_lock);
  waiting_cars[origin]++;
  if (cars_at_intersection == 0) {
    safe_to_go = origin;
  }
  while (safe_to_go != origin) {
    cv_wait(traffic_cv[origin], traffic_lock);
  }
  cars_at_intersection++;
  lock_release(traffic_lock);
  return;
}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination) 
{
  /* replace this default implementation with your own implementation */
  // (void)origin;  /* avoid compiler complaint about unused parameter */
  (void)destination; /* avoid compiler complaint about unused parameter */
  // KASSERT(intersectionSem != NULL);
  // V(intersectionSem);
  lock_acquire(traffic_lock);
  waiting_cars[origin]--;
  int max = 0;
  if (--cars_at_intersection == 0) {
    for (unsigned int i = 0; i < 4; i++) {
      if (waiting_cars[i] > waiting_cars[max]) {
        max = i;
      }
    }
    cv_broadcast(traffic_cv[max], traffic_lock);
    safe_to_go = max;
  }
  lock_release(traffic_lock);
  return;
}
