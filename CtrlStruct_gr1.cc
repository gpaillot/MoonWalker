#include "CtrlStruct_gr1.h"
//#include "namespace_ctrl.h"

//NAMESPACE_INIT(ctrlGr1);

/*! \brief initialize the controller structure
 *
 * \param[in] inputs inputs of the controller
 * \param[in] outputs outputs of the controller
 * \return controller main structure
 */
CtrlStruct* init_CtrlStruct(CtrlIn *inputs, CtrlOut *outputs)
{
    int i;

	CtrlStruct *cvs;

	cvs = (CtrlStruct*) malloc(sizeof(CtrlStruct));

	cvs->inputs  = inputs;
	cvs->outputs = outputs;

    ////// Here, we can implement new structures! Do not forget to add their definition in CtrlStruct_gr1.h! Here, only the initialization //////

    // Structure Tower
    cvs->struct_tower= (StructTower*) malloc(sizeof(StructTower));   // Allocate space

    // For fixed beacons
    cvs->struct_tower->counter=0;
    cvs->struct_tower->tabFixed = (double*) malloc(3*sizeof(double)); // Tabular of doubles of size 3 for the fixed beacons
    cvs->struct_tower->previous_rising_index=0;

    // For opponent

    cvs->struct_tower->counterDist=0;
    cvs->struct_tower->dist=0;
    cvs->struct_tower->previousDistance=0;
    cvs->struct_tower->tabDistance = (double*) malloc(10*sizeof(double)); // Tabular of doubles of size 10

    // Structure for wheels : definition

    cvs->struct_wheels = (StructWheels*) malloc(sizeof(StructWheels));
    cvs->struct_wheels->prev_distance = (double*) malloc(2*sizeof(double));
    cvs->struct_wheels->prev_speed = (double*) malloc(2*sizeof(double));
    cvs->struct_wheels->counter = 0;
    cvs->struct_wheels->x_t = 0.0;
    cvs->struct_wheels->y_t = 0.0;
    cvs->struct_wheels->theta_t = 0.0;


    // initial speed = 0
    cvs->struct_wheels->prev_speed[0] = 0.0;
    cvs->struct_wheels->prev_speed[1] = 0.0;

    // initial position  = 0
    cvs->struct_wheels->prev_speed[0] = 0.0;
    cvs->struct_wheels->prev_speed[1] = 0.0;

    // Structure for the ontroller : definition
    cvs->struct_control = (StructControl *) malloc(sizeof(StructControl));
    cvs->struct_control->sum_error = (double *) malloc(2*sizeof(double));
    cvs->struct_control->Kp = 0.0; //to be computed
    cvs->struct_control->Ki = 0.0; //to be computed

    // initial errors on both wheels = 0; 0 = right wheel; 1 = left wheel
    cvs->struct_control->sum_error[0] = 0.0; // right wheel
    cvs->struct_control->sum_error[1] = 0.0; // left wheel

	return cvs;
}

/*! \brief release controller main structure memory
 *
 * \param[in] cvs controller main structure
 */
void free_CtrlStruct(CtrlStruct *cvs)
{
    free(cvs->inputs);
    free(cvs->outputs);
   

    // Structure tower
    free(cvs->struct_tower->tabFixed);
    free(cvs->struct_tower->tabDistance);
    free(cvs->struct_tower);
    // struct. for wheels
    free(cvs->struct_wheels->prev_speed);
    free(cvs->struct_wheels->prev_distance);
    free(cvs->struct_wheels);
    // struct. for control
    free(cvs->struct_control->sum_error);
    free(cvs->struct_control);
     free(cvs);
    // free the global structure


}
