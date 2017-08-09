#include "Triangle.h"
#include "Math.h"

//=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~//
// Triangle object.                                                       //
//                                                                        //
// The triangle object is defined by three vertices in R3.  This is a     //
// simple flat triangle with no normal vector interpolation.  The         //
// triangle structure is defined to accommodate the barycentric coord     //
// method of intersecting a ray with a triangle.                          //
//=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~//

Triangle::Triangle( const Vec3 &A_, const Vec3 &B_, const Vec3 &C_ )
    {
	float base;		// Length of the base of the triangle
	float height;	// Height of the triangle
	Vec3 N_pos;		// Normal with all values positive

	// Assign values to the corner points
	A = A_;
	B = B_;
	C = C_;

	// Compute the normal to plane of the triangle
	// Cross product between two vectors created by points fo the triangle

    N = Unit( ( C - B ) ^ ( A - B ) );

	// Compute the distance from origin to plane of triangle.
	// This equals to D on the plane coordinates

    d = -( A.x * N.x + A.y * N.y + A.z * N.z );

	// Computes the bounding box;

	// Initiallizes the box coordinates
    box.X.min = box.X.min  = A.x;
    box.Y.min = box.Y.min  = A.y;
    box.Z.min = box.Z.min  = A.z;
	// Check B coordinates
	if( B.x < box.X.min ) box.X.min = B.x;
	else if( B.x > box.X.max ) box.X.max = B.x;
	if( B.y < box.Y.min ) box.Y.min = B.y;
	else if( B.y > box.Y.max ) box.Y.max = B.y;
	if( B.z < box.Z.min ) box.Z.min = B.z;
	else if( B.z > box.Z.max ) box.Z.max = B.z;
	// Check C coordinates
	if( C.x < box.X.min ) box.X.min = C.x;
	else if( C.x > box.X.max ) box.X.max = C.x;
	if( C.y < box.Y.min ) box.Y.min = C.y;
	else if( C.y > box.Y.max ) box.Y.max = C.y;
	if( C.z < box.Z.min ) box.Z.min = C.z;
	else if( C.z > box.Z.max ) box.Z.max = C.z;

	// Compute absolute value for all components of the normal
	N_pos = N;
	N_pos.x = fabs( N_pos.x );
	N_pos.y = fabs( N_pos.y );
	N_pos.z = fabs( N_pos.z );
	
	if( N_pos.x >= N_pos.y && N_pos.x >= N_pos.z )
		axis = 0;
	else if( N_pos.y >= N_pos.x && N_pos.y >= N_pos.z )
		axis = 1;
	else if( N_pos.z >= N_pos.x && N_pos.z >= N_pos.y )
		axis = 2;
	
	// Compute the area

	// First compute the base of the triangle
	base = (float) dist( B, A );

	// Then compute the height of the triangle
	// It is computed as an point to line distance
	height = (float) Length( ( B - A ) ^ ( A - C ) ) / (float) Length( B - A );

	// Therefore, the area is:
	area = base * height * 0.5f;

    next = NULL;
    }

Object *Triangle::ReadString( const char *params ) // Read params from string.
    {
    float ax, ay, az, bx, by, bz, cx, cy, cz;
    if( sscanf( params, "triangle (%f,%f,%f) (%f,%f,%f) (%f,%f,%f)", 
        &ax, &ay, &az, &bx, &by, &bz, &cx, &cy, &cz ) == 9 )
        return new Triangle( Vec3( ax, ay, az ), Vec3( bx, by, bz ), Vec3( cx, cy, cz ) );
    return NULL;
    }

Box3 Triangle::GetBounds() const // Return pre-computed box.
    {
    return box;
    }

/**/
bool Triangle::Intersect( const Ray &ray, HitGeom &hitgeom ) const
{
	
	
	Plane p = Plane(N.x, N.y, N.z, d);
	
	double interSecPoint = p.Intersect(ray);
	Vec3 intersecPointPosition;
	
		/*Origen i destí del raig*/
		Vec3 Ro = ray.origin;
		Vec3 Rd = ray.direction;
		/*Punt d'interseccio*/
		Vec3 Pint;
		/*Vertex del Triangle*/
		Vec3 P1 = A; /*A.x A.y A.z*/
		Vec3 P2 = B; /*B.x B.y B.z*/
		Vec3 P3 = C; /*C.x C.y C.z*/
		Vec3 Bar_Cor;
		/*Matriu per a calcular coordenades baricènctriques*/
		Mat3x3 Minv = Mat3x3();
		Mat3x3 Mat = Mat3x3();
		Mat.m[0][0] = A.x; Mat.m[0][1] = B.x; Mat.m[0][2] = C.x;
		Mat.m[1][0] = A.y; Mat.m[1][1] = B.y; Mat.m[1][2] = C.y;
		Mat.m[2][0] = A.z; Mat.m[2][1] = B.z; Mat.m[2][2] = C.z;

		Mat.m[axis][0] = 1.0f;
		Mat.m[axis][1] = 1.0f;
		Mat.m[axis][2] = 1.0f;
		/*1- Defineixo el pla del triangle -> Necesitem el seu vector normal */
		Plane triangle = Plane(N.x, N.y, N.z, d);

		/*2-dist = Calcul de la distancia entre l'origen del raig i el pla*/
		/*angle entre vector direccio del raig i el vector distancia minima*/
		float angle = acos(((-N.x*Rd.x) - (N.y*Rd.y) - (N.z*Rd.z)) /
			(sqrt(N.x*N.x + N.y*N.y + N.z*N.z)*sqrt(Rd.x*Rd.x + Rd.y*Rd.y + Rd.z*Rd.z)));
		/*Distancia minima entre punt origen i el pla*/
		float dist = abs(N.x*(Ro.x) + N.y * (Ro.y) + N.z *(Ro.z) + d) /
			(sqrt(N.x*N.x + N.y*N.y + N.z* N.z));
		/*trigonometria cos(angle)=contigu/hipot*/
		float Dint = dist / cos(angle);

		Dint = triangle.Intersect(ray);


		/*3- Si dist > 0 && dist < distancia_Xoc  */
		if (Dint > 0 && Dint < hitgeom.distance){
			/*Busca el punt P del pla que conte el triangle*/
			Pint = Ro + Dint * Rd;
			
			if (axis == 0) Pint.x = 1.0f;
			if (axis == 1) Pint.y = 1.0f;
			if (axis == 2) Pint.z = 1.0f;


			/*Calcula les coordenades baricèntriques*/
			Minv = (1 / det(Mat)) * Transpose(Adjoint(Mat));
			Bar_Cor = Minv * (Vec3(Pint.x, Pint.y, Pint.z));
			/*Les coordenades son a dins el triangle?*/
			if ((0.0f <= Bar_Cor.x) && (Bar_Cor.x <= 1.0f) && (0.0f <= Bar_Cor.y) && (Bar_Cor.y <= 1.0f) && (0.0f <= Bar_Cor.z) && (Bar_Cor.z <= 1.0f)){
				hitgeom.distance = Dint;
				hitgeom.normal = N;
				hitgeom.origin = Ro;
				hitgeom.point = Pint;
				return true;
			}
		}

	
	return false;
	}

//Implementar
Sample Triangle::GetSample( const Vec3 &P, const Vec3 &N_point ) const
{
	double s, t; //Random coordinates
	Sample sample;       //P= Point & W = weight
	double cos_theta;

	// Generamos dos coordenadas aleatorias para t y s
	double d = 1.0;
	double projected_area;
	
	// Generates two random values for s and t coordinates
	s = rand(0.0, 1.0);
	t = rand(0.0, 1.0);
	float raizS = sqrt(s);
	// Adds the new position
	sample.P = (1 - raizS) * A + raizS * (1 - t) * B + t * raizS * C;
	// Cosine of the angle between the light ray and the normal at the light source
	cos_theta = fabs(N * Unit(P - sample.P));
	// Calcule the distance between the sample and the point P
	d = Length(sample.P - P);
	// Assigns the weight &&  Controlar que no sigui major a 2*Pi
	
		sample.w = area * cos_theta / (d * d);  
		sample.w = sample.w > 2 * Pi ? 2 * Pi : sample.w;
	
	return sample;
}