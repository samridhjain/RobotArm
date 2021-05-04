// Current location, extract from CSV
// next point, csv extract
// can arm reach? if no, step 2
// shortest path?? if no, step 2... easier would be to log all and then sort path by distance
// distance required to move, x-y motion
// degree rotation for the point, Z axis
// orientation of R1,R2 by t1 and t2 (t is theta)
// Reach for the point
// Log the motion, repeat from step 2 until N = 13
// Sort by distance of x or y axis
// print log

#define N 13 // number of entries
#define _USE_MATH_DEFINES // insert this to use math constants in the program


#include<stdio.h>
#include<math.h>


//define structure of dots in cartesian system
// https://www.tutorialspoint.com/cprogramming/c_typedef.htm

typedef struct position {
    int num;
    double x;
    double y;
    double z;    
}position;


void scan(int a, FILE* fp1, position* position){
    fscanf(fp1, "%d,%lf,%lf,%lf\n", &position->num, &position->x, &position->y, &position->z);
}

void output(int a, position position){
    printf("%2d		%8.2f	%7.2f	%7.2f  ", position.num, position.x, position.y, position.z);
}

int evaluate(int i, position position, int* pt1, int* pt2){
    
    // assign variable to each x,y,z value from struct for calculation
    double x = position.x;
    double y = position.y;
    double z = position.z;

    double ptx; //closest x of point
    double pty = 0; //closest y of point

    int dx = 0; //∂ in x and ptx
    int dy = 0; //∂ in y and pty
    double ex = 0;
	double ey = 0;
	double d;
	int g;

    if (x<=1000){
        ptx =1000;
    }
    else if (x>= 9000){
        ptx= 9000;
    }
    else{
       	dx = x / 1000;
		ex = x - dx * 1000;
		if (ex <= 500){
			ptx = (double)dx * 1000;
		}
		else{
			ptx = ((double)dx + 1) * 1000;
		}
	} 

    if (y <= 1000) {
		pty = 1000;
	}
	else if (y >= 9000.0) {
		pty = 9000;
	}
	else {
		dy = y / 1000;
		ey = y - (double)dy * 1000;
		if (ey <= 500) {
			pty = (double)dy * 1000;
		}
		else {
			pty = ((double)dy + 1) * 1000;
		}
	}

	d = (x - ptx) * (x - ptx) + (y - pty) * (y - pty) + (z - 100) * (z - 100);

	if(ex == 0 && ey == 0 && z == 0){
		g = 0;
	}
	else if(d <= 810000){
		g = 1;
	}
	else{
		g = 0;
	}

	pt1[i] = ptx;
	pt2[i] = pty;

	return g;

}

void machinedegree(int i, position position, int pt1[], int pt2[], double* p3){
    double x = position.x;
	double y = position.y;
	double z = position.z;

	int ptx = pt1[i];
	int pty = pt2[i];
	double mdeg;

	if ((x-ptx)==0 && (y-pty) == 0){
		mdeg = 0.00; //machine doesnt need to rotate
	}
	else if ((x-ptx)<0 && (y-pty)>0){
		mdeg = (atan((y-pty)/(x-ptx))/M_PI)*180 + 180;  
	}
	else if ((x-ptx)<0 && (y-pty)<0){
	mdeg = (atan((y-pty)/(x-ptx))/M_PI)*180 - 180;  
	}
	else{
		mdeg = (atan((y-pty)/(x-ptx))/M_PI)*180;
	}
	p3[i] = mdeg;
	
	return;
}

void deg(int i, position position, int pt1[], int pt2[], double* pt3,double* pt4){
    double x = position.x;
	double y = position.y;
	double z = position.z;
	
	int ptx = pt1[i]; //position of desired point x
	int pty = pt2[i]; //position of desired point y
	
	double t1;
	double t2; 

	double l1 = 500; //arm length of first 
	double l2 = 400; //arm length of second
	double r;

	double alpha;
	double beta;
	double phi;

	double a,b; 
	a = x - ptx;
	b = y - pty;
	r = sqrt((a*a)+(b*b)+(z-100)*(z-100)); 
	
	// INVERSE KINEMATICS
	alpha = acos(((l1 * l1) + (r*r) - (l2 * l2)) / (2 * l1 * r)) * (180 / M_PI); // finding the angle of the first arm
	beta = acos(((l1 * l1) + (l2 * l2) - (r*r)) / (2 * l1 * l2)) * (180 / M_PI); // finding the angle of the second arm 

	if(isnan(alpha)){
		printf("%.2f \n", (l1 * l1 + r*r - l2 * l2) / (2 * l1 * r));
	}
	if(isnan(beta)){
		printf("%.2f \n", (l1 * l1 + l2 * l2 - r) / (2 * l1 * l2));
	}
	phi = (atan((z-100)/sqrt(a*a)+(b*b))/M_PI)*180;
	
	//Thetas
	t1= phi - alpha;
	t2 = 180 - beta;

	printf(" %.2f %.2f \n", t1, t2);
	


	pt3[i] = t1;
	pt4[i] = t2;

	return;
}

//https://www.codingunit.com/bubble-sort-algorithm
// need a swap function to make it convenient later
void swap(int* x, int* y){//swap function used in bubble sort
//swap elements x and y for t1,t2,t3	
	int temp = *x;
	*x = *y;
	*y = temp;
}

void sort(int* pt1, int* pt2, int* pt3, int k, double* pt4,double* pt5,double* pt6){
//store each address in int t
	int* t1 = pt1;
	int* t2 = pt2;
	int* t3 = pt3;
	int i;

//Bubble Sort
//to put it in order, we need to swap when t1 is greater or equal to t1+1. Else, do nothing.

	while(1){
		
		int swap_flag = 1; //use flag variable to exit loop after sorting
		for( i = 0; i < k-1; i++){
			if (t1[i] > t1[i+1]){
					swap_flag = 0;
					//swap each t
					swap(&t1[i],&t1[i+1]);
					swap(&t2[i],&t2[i+1]);
					swap(&t3[i],&t3[i+1]);
				}
		}

		if (swap_flag== 1){
			break; //swap occurred
		}
	}

	while(1){
		int swap_flag = 1; //use flag variable to exit loop after sorting
		for( i = 0; i < k-1; i++){
			//when the destination i+1 is same as current i 
			if (t1[i] == t1[i+1]){

				int m = (t1[i]/1000)%2;

				if(m!=0){
					if (t2[i] >  t2[i + 1]) {
						swap_flag = 0;
						swap(&t1[i], &t1[i + 1]);
						swap(&t2[i], &t2[i + 1]);
						swap(&t3[i], &t3[i + 1]);
					}
				}
				else{
					if (t2[i] <t2[i + 1]) {
						swap_flag = 0;
						swap(&t1[i], &t1[i + 1]);
						swap(&t2[i], &t2[i + 1]);
						swap(&t3[i], &t3[i + 1]);
					}
				}
			}

		}
		if (swap_flag == 1){
			break;
		}
	}
}

//Function to create a log at the end of travel
void Log(int* pt1, int* pt2, int k, double* pt3, double* pt4, double* pt5, FILE* fp2){
	int i;
	int j1 = 0;
	int j2 = 0;
	int x1;
	int y1;
	int x2;
	int y2;
	int a;
	int a1;
	int b;
	int b1;

	while (1) {

		for (j1 = 0; j1 < k; j1++) {

			if (j1 == 0) {

				x1 = 1000;
				y1 = 1000;
				x2 = pt1[j1];
				y2 = pt2[j1];

				if (x1 != x2) {
					b = (x2 - x1) / 1000;

					for (j2 = 0; j2 < b; j2++) {

						printf("( %4d, %4d)  ", x2, y1);
						printf("( %4d, %4d)", j2 * 1000, 0);
						printf("     0.00 deg      90.00 deg       -90.00 deg");
						printf("   N/A\n");
						fprintf(fp2, "%4d,%4d,%4d,0.00,0.00deg,90.00deg,-90.00deg,N/A\n ", x2, y1, j2 * 1000);
					}

					a = (y2 - y1) / 1000;

					for (j2 = 0; j2 < a; j2++) {

						printf("( %d, %d)  ", x2, y1 + j2 * 1000);
						printf("( %5d, %4d)", x2 - x1, j2 * 1000);
						fprintf(fp2, "%4d,%4d,%4d,%4d, ", x2, y1 + j2 * 1000, x2 - x1, j2 * 1000);
						if (j2 != a - 1) {
							printf("    0.00 deg      90.00 deg      -90.00 deg");
							printf("   N/A\n");
							fprintf(fp2, "0.00deg,90.0deg,-90.00deg,N/A\n");
						}
						else {
							printf("  %3.2f deg     %3.2f deg    %3.2f deg", pt3[0], pt4[0], pt5[0]);
							printf(" Reaching\n");
							fprintf(fp2, "%3.2f deg,%3.2f deg,%3.2f deg, Reaching\n", pt3[0], pt4[0], pt5[0]);
						}
					}
				}
				else {
					a = (y2 - y1) / 1000 + 1;

					for (j2 = 0; j2 < a; j2++) {

						printf("( %d, %d)  ", x2, y1 + j2 * 1000);
						printf("( %5d, %4d)", x2 - x1, j2 * 1000);
						fprintf(fp2, "%4d,%4d,%4d,%4d, ", x2, y1 + j2 * 1000, x2 - x1, j2 * 1000);
						if (j2 != a - 1) {
							printf("    0.00 deg     90.00 deg    -90.00 deg");
							printf("   N/A\n");
							fprintf(fp2, "0.00deg, 90.0deg,-90.00deg,N/A\n");
						}
						else {
							printf("  %3.2f deg     %3.2f deg     %3.2f deg", pt3[0], pt4[0], pt5[0]);
							printf(" Reaching\n");
							fprintf(fp2, "%3.2f deg,%3.2f deg,%3.2f deg, Reaching\n", pt3[0], pt4[0], pt5[0]);

						}
					}

				}
			}
			else {
				x1 = pt1[j1 - 1];
				y1 = pt2[j1 - 1];


				x2 = pt1[j1];
				y2 = pt2[j1];

				if (x1 != x2) {
					b = (x2 - x1) / 1000;

					for (j2 = 0; j2 < b; j2++) {

						printf("( %d, %4d)  ", x1 + (j2 + 1) * 1000, y1);
						printf("( %5d,    0)", (j2 + 1) * 1000);
						printf("  %3.2f deg     %3.2f deg     %3.2f deg", pt3[j1 - 1], pt4[j1 - 1], pt5[j1 - 1]);
						printf("   N/A\n");
						fprintf(fp2, "%d,%4d,%5d, 0,%3.2f deg,%3.2f deg,%3.2f deg,N/A\n", x1 + (j2 + 1) * 1000, y1, (j2 + 1) * 1000, pt3[j1 - 1], pt4[j1 - 1], pt5[j1 - 1]);

					}


					a = (y2 - y1) / 1000;

					if (a > 0) {

						for (j2 = 0; j2 < a; j2++) {

							printf("( %d, %d)  ", x2, y1 + (j2 + 1) * 1000);
							printf("(     0,%5d)", (j2 + 1) * 1000);
							fprintf(fp2, "%d,%d,0,%5d,", x2, y1 + (j2 + 1) * 1000, (j2 + 1) * 1000);
							if (j2 != a - 1) {
								printf("  %3.2f deg     %3.2f deg     %3.2f deg", pt3[j1 - 1], pt4[j1 - 1], pt5[j1 - 1]);
								printf("   N/A\n");
								fprintf(fp2, "%3.2f deg,%3.2f deg,%3.2f deg,N/A\n", pt3[j1 - 1], pt4[j1 - 1], pt5[j1 - 1]);
							}
							else {
								printf("  %3.2f deg     %3.2f deg     %3.2f deg", pt3[j1], pt4[j1], pt5[j1]);
								printf(" Reaching\n");
								fprintf(fp2, "%.2f deg,%3.2f deg,%3.2f deg, Reaching\n", pt3[j1], pt4[j1], pt5[j1]);
							}
						}
					}
					else {
						a = -1 * a;

						for (j2 = 0; j2 < a; j2++) {

							printf("( %d, %d)  ", x2, y1 - (j2 + 1) * 1000);
							printf("(     0,%5d)", -1 * (j2 + 1) * 1000);
							fprintf(fp2, "%d,%d,0,%d,", x2, y1 - (j2 + 1) * 1000, -1 * (j2 + 1) * 1000);

							if (j2 != a - 1) {
								printf("  %3.2f deg     %3.2f deg     %3.2f deg", pt3[j1 - 1], pt4[j1 - 1], pt5[j1 - 1]);
								printf("   N/A\n");
								fprintf(fp2, "%3.2f deg,%3.2f deg,%3.2f deg,N/A\n", pt3[j1 - 1], pt4[j1 - 1], pt5[j1 - 1]);
							}
							else {
								printf("  %3.2f deg     %3.2f deg     %3.2f deg", pt3[j1], pt4[j1], pt5[j1]);
								printf(" Reaching\n");
								fprintf(fp2, "%3.2f deg,%3.2f deg,%3.2f deg,Reaching\n", pt3[j1], pt4[j1], pt5[j1]);
							}

						}

					}


				}
				else {
					a = (y2 - y1) / 1000;

					if (a > 0) {

						for (j2 = 0; j2 < a; j2++) {

							printf("( %d, %d)  ", x2, y1 + j2 * 1000);
							printf("(     0,%5d)", j2 * 1000);
							fprintf(fp2, "%d,%d,0,%5d,", x2, y1 + j2 * 1000, j2 * 1000);

							if (j2 != a - 1) {
								printf("  %3.2f deg     %3.2f deg     %3.2f deg", pt3[j1 - 1], pt4[j1 - 1], pt5[j1 - 1]);
								printf("   N/A\n");
								fprintf(fp2, "%3.2f deg,%3.2f deg,%3.2f deg,N/A\n", pt3[j1 - 1], pt4[j1 - 1], pt5[j1 - 1]);
							}
							else {
								printf("  %3.2f deg     %3.2f deg     %3.2f deg", pt3[j1], pt4[j1], pt5[j1]);
								printf(" Reaching\n");
								fprintf(fp2, "%3.2f deg,%3.2f deg,%3.2f deg,Reaching\n", pt3[j1], pt4[j1], pt5[j1]);
							}
						}
					}
					else {
						a = -1 * a;

						for (j2 = 0; j2 < a; j2++) {

							printf("( %d, %d)  ", x2, y1 - (j2 + 1) * 1000);
							printf("(     0,%5d)", -1 * (j2 + 1) * 1000);
							fprintf(fp2, "%d,%d,0,%5d,", x2, y1 - (j2 + 1) * 1000, -1 * (j2 + 1) * 1000);
							if (j2 != a - 1) {
								printf("  %3.2f deg     %3.2f deg     %3.2f deg", pt3[j1 - 1], pt4[j1 - 1], pt5[j1 - 1]);
								printf("   N/A\n");
								fprintf(fp2, "%3.2f deg,%3.2f deg,%3.2f deg,N/A\n", pt3[j1 - 1], pt4[j1 - 1], pt5[j1 - 1]);
							}
							else {
								printf("  %3.2f deg     %3.2f deg     %3.2f deg", pt3[j1], pt4[j1], pt5[j1]);
								printf(" Reaching\n");
								fprintf(fp2, "%3.2f deg,%3.2f deg,%3.2f deg,Reaching\n", pt3[j1], pt4[j1], pt5[j1]);
							}
						}

					}

				}

			}

		}

		break;

	}

	x2 = pt1[k - 1];
	y2 = pt2[k - 1];

	a1 = (y2 - 1000) / 1000;

	if (a1 != 0) {
		for (i = 1; i < a1 + 1; i++) {
			printf("( %d, %d)  ", x2, y2 - 1000 * i);
			printf("(    0, %d)", -1000 * i);
			printf("  %3.2f deg     %3.2f deg     %3.2f deg", pt3[k - 1], pt4[k - 1], pt5[k - 1]);
			printf("   N/A\n");
			fprintf(fp2, "%d,%d,0,%d,%3.2f deg,%3.2f deg,%3.2f deg,N/A\n", x2, y2 - 1000 * i, -1000 * i, pt3[k - 1], pt4[k - 1], pt5[k - 1]);
		}
	}

	b1 = (x2 - 1000) / 1000;

	if (b1 != 0) {
		for (i = 1; i < b1 + 1; i++) {
			printf("( %d, 1000)  ", x2 - i * 1000);
			printf("( %d,    0)", -1000 * i);
			printf("  %3.2f deg     %3.2f deg     %3.2f deg", pt3[k - 1], pt4[k - 1], pt5[k - 1]);
			printf("   N/A\n");
			fprintf(fp2, "%d,1000,%d,0, %3.2f deg , %3.2f deg , %3.2f deg ,N/A\n", x2 - i * 1000, -1000 * i, pt3[k - 1], pt4[k - 1], pt5[k - 1]);
		}
	}
	return;
}

int main(void) {
	position position[N];

	FILE* fp1= fopen("coordinates.csv", "r"); // the file with the coordinate inputs
	FILE* fp2 = fopen("log.csv", "w"); // the file with the positions that are accesible

	char tag[4][10];
	int i;
	int j;
	int k = 0;
	int l = 0;
	//begin data of X and Y from 0th value, initial value
	int ActX[N] = {0};
	int ActY[N] = {0};

	double mdeg[N] = {0}; // rotation of the robot
	double a1deg[N] = {0}; //degree of each joint
	double a2deg[N] = {0}; //degree of joint 2
	double mdeg2[N] = {0}; //only the ones that reach
	double a1deg2[N] = {0}; //degree of joint that only reaches
	double a2deg2[N] = {0}; // degree of joint 2 that only reaches 
	int okng[N] = {0}; // 

	int* pt1 = ActX;
	int* pt2 = ActY;
	double* pt3 = mdeg;
	double* pt7 = mdeg2;
	double* pt8 = a1deg2;
	double* pt9 = a2deg2;
	double* pt10 = a1deg;


	//reading the csv file with the point inputs
	//read from top line
	fscanf(fp1, "%[^,],%[^,],%[^,],%s\n", tag[0], tag[1], tag[2],tag[3]);

	printf("\n");

	//scan csv values and assign with the structure defined
	for (i = 0; i < N; i++) {
		scan(i, fp1, &position[i]);
	}

	printf("CSV file has been read!\n\n");
	printf("%d Destination Points scanned! \n\n", N);

	//call output function to display the values scanned
	for (i = 0; i < 4; i++) {
		printf("%s ", tag[i]);
		if (i == 3) {
			printf("\n");
		}
	}
	//Print the coordinates of the position of points
	for (i = 0; i < N; i++) {
		output(i, position[i]);
		printf("\n");
	}

	printf("\n\n");
	printf("\n");

	for (i = 0; i < N; i++) {

		j = evaluate(i, position[i], ActX, ActY);
		if (j == 1) {

			okng[i] = 1;
			k++;
		}
		else {
			okng[i] = 0;
		}
	}
	//print validity of data
	printf("%d The data points are in range\n\n", k);

	//Reachable points
	int num_def[N] = { 0 };
	int ActX1[N] = { 0 };
    int ActY1[N] = { 0 };

	int* pt4 = ActX1;
	int* pt5 = ActY1;
	int* pt6 = num_def;

	printf("No.\t X\t\t Y\t Z\tJudge\t XAct\t ActY\t MDeg\t A1Deg\t A2Deg\t\n");


	// for (i = 0; i < 4; i++) {
	// 	printf("%s ", tag[i]);
	// 	if (i == 3) {
	// 		printf("\n");
	// 	}
	// }

	for (i = 0; i < N; i++) {
		output(i, position[i]);


		if (okng[i] == 1) {
			printf("OK\t");
			printf(" %d  %d ", ActX[i], ActY[i]);
			machinedegree(i, position[i], ActX, ActY, mdeg);
			printf(" %3.2f", mdeg[i]);
			deg(i, position[i], ActX, ActY, a1deg, a2deg);

			//For reachable points
			if (ActX[i] == 0) {
				num_def[l] = i + 1;
				ActX1[l] = i + 1;
				ActY1[l] = 1;
				l++;
			}
			else {
				num_def[l] = i + 1;
				ActX1[l] = ActX[i];
				ActY1[l] = ActY[i];
				mdeg2[l] = mdeg[i];
				a1deg2[l] = a1deg[i];
				a2deg2[l] = a2deg[i];
				l++;
			}
		}
		else {
			printf("NG");
			printf("\t -      -        -        -       - \n");
		}
	}
	printf("\n\n");

	//Confirmed Output
	for (i = 0; i < N; i++) {
		if (ActX1[i] != 0) {
			printf("%2d  %d  %d  %3.2f  %3.2f  %3.2f\n", num_def[i], ActX1[i], ActY1[i], mdeg2[i], a1deg2[i], a2deg2[i]);
		}
	}
	printf("\n\n");

	sort(pt4, pt5, pt6, k, pt7, pt8, pt9);
	printf("\n");

	printf("Evaluated the order of reaching for maximum efficiency based on distance of x co-ordinate.\n\n");

	printf("\n\n");

	for (i = 0; i < N; i++) {

		if (ActX1[i] != 0) {
			printf("%2d  %d  %d  %3.2f  %3.2f  %3.2f\n", num_def[i], ActX1[i], ActY1[i], mdeg2[i], a1deg2[i], a2deg2[i]);
		}
	}

	printf("\n\n Log: \n\n");

	printf("AbsCoordinate  RelCoordinate    MachineDeg    ArmDeg1      ArmDeg2      ActState\n");
	fprintf(fp2, "Coordinate X ,Coordinate Y, ∂x, ∂y,   Rotation of arm 1  , Rotation of arm 2  ,    ActState\n");

	Log(pt4, pt5, k, pt7, pt8, pt9, fp2);

	printf("Data logged in new file: log.csv\n");

	fclose(fp1);
	fclose(fp2);

	return 0;
}