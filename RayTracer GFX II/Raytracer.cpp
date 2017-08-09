#include <math.h>
/***************************************************************************
*                                                                          *
* This is the source file for a ray tracer. It defines most of the		   *
* fundamental functions using in ray tracing.  In particular, "Trace" and  *
* "Shade".  It also defines the MakeImage function, which casts all the    *
* rays from the eye, and several object intersection methods.  Some of the *
* functions are left blank, or with "place holders" that don't do very     *
* much.  You need to fill these in.                                        *
*                                                                          *
*                                                                          *
***************************************************************************/

static const int tree_depth = 3;		// Number of recursions to compute indirect illumination
static const int rays_pixel = 10;

#include "Raytracer.h"


// Draw image on the screen
void Raytracer::draw( void )
{
	glDrawPixels( resolutionX, resolutionY, GL_RGB, GL_UNSIGNED_BYTE, &(*I)( 0 , 0 ) );
}

// Cast_line casts all the initial rays starting from the eye for a single
//  raster line. Copies pixels to image object.
void Raytracer::cast_line(World world)
{
	Ray ray;
	Color color;	// Color computed when using multiple rays per pixel

	ray.origin = world.getCamera().eye; // All initial rays originate from the eye.
	ray.no_emitters = false;

	Vec3 G = Unit(world.getCamera().lookat - world.getCamera().eye);	// Gaze direction.
	Vec3 U = Unit(world.getCamera().up / G);							// Up vector.
	Vec3 R = Unit(G ^ U);											// Right vector.
	Vec3 O = (world.getCamera().vpdist * G) - R + U;					// "Origin" for the raster.
	Vec3 dU = U * (2.0 / (resolutionY - 1));						// Up increments.
	Vec3 dR = R * (2.0 / (resolutionX - 1));						// Right increments.

	if (currentLine % 10 == 0) cout << "line " << currentLine << endl;
	for (int i = 0; i < resolutionX; i++)
	{
		if (rays_pixel == 1)
		{
			// One ray per pixel
			ray.direction = Unit(O + i * dR - currentLine * dU);
			color = Trace(ray, world.getScene(), tree_depth);
		}
		else
		{
			// Multisampling
			for (int n = 0; n < rays_pixel; n++)
			{
				ray.direction = Unit(O + (i + rand(0.0, 1.0) - 0.5) * dR - (currentLine + rand(0.0, 1.0) - 0.5) * dU);
				color += Trace(ray, world.getScene(), tree_depth);
			}
		}
		(*I)(resolutionY - currentLine - 1, i) = ToneMap(color / rays_pixel);

		color.blue = 0;
		color.green = 0;
		color.red = 0;
	}

	if (++currentLine == resolutionY)
	{
		// Image computation done, save it to file
		cout << "done." << endl;
		I->Write("Resultat.ppm");
		isDone = true;
	}
}


// This is a trivial tone mapper; it merely maps values that are
// in [0,1] and maps them to integers between 0 and 255.  If the
// real value is above 1, it merely truncates.  A true tone mapper
// would attempt to handle very large values nicely, without
// truncation; that is, it would try to compensate for the fact that
// displays have a very limited dynamic range.
Pixel Raytracer::ToneMap( const Color &color )
{
	int red   = (int)floor( 256 * color.red   );
    int green = (int)floor( 256 * color.green );
    int blue  = (int)floor( 256 * color.blue  );
    channel r = (channel)( red   >= 255 ? 255 : red   ); 
    channel g = (channel)( green >= 255 ? 255 : green ); 
    channel b = (channel)( blue  >= 255 ? 255 : blue  );
    return Pixel( r, g, b );
}

// Trace is the most fundamental of all the ray tracing functions.  It
// answers the query "What color do I see looking along the given ray
// in the current scene?"  This is an inherently recursive process, as
// trace may again be called as a result of the ray hitting a reflecting
// object.  To prevent the possibility of infinite recursion, a maximum
// depth is placed on the resulting ray tree.
Color Raytracer::Trace( const Ray &ray, const Scene &scene, int max_tree_depth  )
{
    Color   color;                    // The color to return.
    HitInfo hitinfo;                  // Holds info to pass to shader.

	// Intitallizes hit distance to infinity to allow finding intersections in all ray length
	hitinfo.geom.distance = Infinity;

	if (Cast( ray, scene, hitinfo ) > 0.0f && max_tree_depth > -1 )
	{
        // The ray hits an object, so shade the point that the ray hit.
        // Cast has put all necessary information for Shade in "hitinfo".
		
		// If the ray has no_emitters activated and the first hit is an emitter
		//  this ray shouldn't contribute to the color of the current pixel
		if( hitinfo.material.Emitter() && ray.no_emitters == true ) color = Color ();

		// The ray hits an object, so shade the point that the ray hit.
        // Cast has put all necessary information for Shade in "hitinfo".
		else color = Shade( hitinfo, scene, max_tree_depth - 1  );
    }
    else
    {
        // Either the ray has failed to hit anything, or
        // the recursion has bottomed out.
        color = scene.bgcolor;
    }
    
    return color;
}

// Cast finds the first point of intersection (if there is one)
// between a ray and a list of geometric objects.  If no intersection
// exists, the function returns false.  Information about the
// closest object hit is returned in "hitinfo". 
int Raytracer::Cast( const Ray &ray, const Scene &scene, HitInfo &hitinfo, Object *ignore )
{
	int hit = false;

    // Each intersector is ONLY allowed to write into the "HitGeom"
    // structure if it has determined that the ray hits the object
    // at a CLOSER distance than currently recorded in HitGeom.distance.
    // When a closer hit is found, the material fields of the "HitInfo"
    // structure are updated to hold the material of the object that 
    // was just hit.

    for( Object *object = scene.first; object != NULL; object = object->next )
    {
        if( object != ignore && object->Intersect( ray, hitinfo.geom ) )
            {
            hitinfo.material = object->material;  // Material of closest surface.
            hit = true;                           // We have hit an object.
            }
    }
    return hit;
}

// Shade assigns a color to a point on a surface, as it is seen
// from another point.  The coordinates of these points, the normal
// of the surface, and the surface material are all recorded in the
// HitInfo structure.  The shader will typically make calls to Trace
// to handle shadows and reflections.
Color Raytracer::Shade(const HitInfo &hit, const Scene &scene, int max_tree_depth)
{
	
	Color color = Color(0,0,0);
	Color Kd = hit.material.m_Diffuse; //Componente difusa
	Color Ks = hit.material.m_Specular; //Componente especular
	float Ko = hit.material.m_Opacity;
	float Fo = hit.material.m_Reflectivity;
	Color Difuse;	//Color Difuso PHONG  
	Color Specular; //Color Especular PHONG  V-> inter a hit.geom.origin
	Color Irradiance;
	Color color_Dir   =	Color{ 0, 0, 0};
	Color color_InDir = Color{ 0, 0, 0 };
	Color difuseReflection, specularReflection;
	float n = hit.material.m_Phong_exp;
	Vec3 N = Unit(hit.geom.normal);
	Ray RayEmiter;  //Rayo creado para un objeto emisor de luz
	Ray RayHemisphere; //Rayo en direccion al Sample del hemisferio
	Ray RaySpecularLobe; //Rayo en direccion al Sample del hemisferio
	HitInfo hitInfoAux; //Informacion sobre la intersection del RayoEmiter y pointSample
	Sample pointSample; //Punto de con una muestra de luz
	Sample HemisphereSample; //Muestra del hemisferio proyectado
	Sample SpecularLobeSample; //Muestra del hemisferio Especular
	Vec3 hitPoint = hit.geom.point + hit.geom.normal * Epsilon; //Punt d'interseccio "inter"
	Vec3 V = Unit(hit.geom.origin - hitPoint); //Vector punt-Camera
	Vec3 R,L; //Vector de reflexio i Vector de Light

	
	if (hit.material.Emitter()){  // Si hemos chocado con una luz
		color = hit.material.m_Emission;
		return color;   //return el color que emite
	}
	else{	//Si no hemos chocado con una luz			
		for (Object *object = scene.first; object != NULL; object = object->next)  //Pera todos los objectos de la escena
		{
			Difuse = { 0, 0, 0 };
			Specular = { 0, 0, 0 };

			if (object->material.Emitter()){    //Si el objecto emite luz
				pointSample = object->GetSample(hitPoint, hit.geom.normal);   //Extraemos un punto de luz con obj->GetSample
				L = Unit(pointSample.P - hitPoint);
				RayEmiter.origin = hitPoint;  //Construimos un rayo hacia este punto y a la direccion calculada (punto-origen)
				RayEmiter.direction = L;
				hitInfoAux.geom.distance = dist(pointSample.P, hitPoint);

				if (!Cast(RayEmiter, scene, hitInfoAux, object)){   	//Si no hay interseccion entre rayo - punto
					Irradiance = pointSample.w * object->material.m_Emission; //Irradiance = Area * object->material.m_Emission;
					Difuse = (1 / Pi)*Kd * fmax(0, (N*L)); //Calculo de componente difusa Phong (l=raig LLUM-PUNT)  
					R = Unit(Reflection(-L, N));
					if (hit.material.m_Phong_exp > 0.0)
					{
						//
						Specular = (Ks*(pow(fmaxf(0, (V*R)), n))); //Calculo de componente especular de Phong 
						//

						/********************TORRANCE SPARROW************************/
						/* float F =  Fo +(1 - Fo)*pow((1-(L*H)),5); // Como es dot(L,H) en c++?  Que es Fo=reflectividad en angulo 0
						/* H = Unit(L+H);
						/*	Vec3 h = Unit(Reflection(V,L));
						/* float D = ((object->material.m_Type + 2) / TwoPi)* pow(fmaxf(0, (h*n)), object->material.m_Type); xk no pones phong exp en vez de m_type?
						/* double G = fminf(double(1.00f), (2 * (n*h)*(n*V)) / (V*h));
						/* float G = fminf(G, (2 * (n*h)*(n*L)) / (L*h)); // Hacemos dos veces el min porque no encontramos un fminf para 3 variables
						/* Specular = (F*D*G) / (4 * (N*L)*(N*V));
						/********************************************/
						Color F;
						float D, G;
						Vec3 H;
						H = Unit(V + L); 
						float NH, NV, VH, NL, LH;
						NH = N*H;
						NV = N*V;
						VH = V*H;
						NL = N*L;
						LH = L*H;

						Color F0 = Ks;//pow(((1 - hit.material.m_RefractiveIndex)/(1 + hit.material.m_RefractiveIndex)),2);// L*N;// hit.material.m_Reflectivity;
						F = F0 + Color(1 - F0.red,1-F0.green,1-F0.blue)*pow(1-VH,5);
						//F = 1;
						/*if((H*N)>0)*/ D = ((n + 2.0f) / TwoPi) * pow(fmax(0,NH),n);
						G = fmin(1.00f,(2*NH*NV)/VH);
						G = fmin(G,(2*NH*NL)/LH);
						
						//Specular = Ks*(F*D*G) / (4*NL*NV);
					}
					color_Dir += (Difuse + Specular)* Irradiance;
				}
			}
		}
		

		difuseReflection = { 0, 0, 0 };
		HemisphereSample = SampleProjectedHemisphere(N); //Extraemos una muestra del hemisferio proyectado
		RayHemisphere.direction = Unit(HemisphereSample.P); //Construimos un rayo en direccion a la muestra 
		RayHemisphere.origin = hitPoint;
		RayHemisphere.no_emitters = true; //Evitamos que recalcule la luz directa
		difuseReflection = HemisphereSample.w*(1 / Pi)*Trace(RayHemisphere, scene, max_tree_depth) *(Kd); //Calculamos la reflexion difusa
		
		specularReflection = { 0, 0, 0 };
		if (n>0) {

			Color F;
			float D, G;
			Vec3 H;
			H = Unit(V + L);
			float NH, NV, VH, NL, LH;
			NH = N*H;
			NV = N*V;
			VH = V*H;
			NL = N*L;
			LH = L*H;

			Color F0 = Ks;//pow(((1 - hit.material.m_RefractiveIndex)/(1 + hit.material.m_RefractiveIndex)),2);// L*N;// hit.material.m_Reflectivity;
			F = F0 + Color(1 - F0.red, 1 - F0.green, 1 - F0.blue)*pow(1 - VH, 5);
			//F = 1;
			/*if((H*N)>0)*/ D = ((n + 2.0f) / TwoPi) * pow(fmax(0, NH), n);
			G = fmin(1.00f, (2 * NH*NV) / VH);
			G = fmin(G, (2 * NH*NL) / LH);

			
			Vec3 Rind = Unit(Reflection(-V, N));//recalcular R como rayo de reflexion ideal de vision
			SpecularLobeSample = SampleSpecularLobe(Rind, n); //Extraigo una muestra del lobulo especular
			//Construyo un rayo con direccion a la muestra
			RaySpecularLobe.direction = SpecularLobeSample.P;
			RaySpecularLobe.origin = hitPoint;
			RaySpecularLobe.no_emitters = true;
			//specularReflection = (Ks*Trace(RaySpecularLobe, scene, max_tree_depth) * (F*G) )/ (4 * NL*NV);

			specularReflection = SpecularLobeSample.w * ((n + 2.0) / TwoPi)*Trace(RaySpecularLobe, scene, max_tree_depth)*Ks; //Calculo de la reflexion especular apartado 2
		}
		color_InDir = difuseReflection +specularReflection;
		color = color_Dir +color_InDir;  //return color directo + color indierecto
	}
	return color;
}


//Devuelve una muestra en el hemisferio proyectado. Este es un tipo de muestreo por importancia
Sample Raytracer::SampleProjectedHemisphere(const Vec3 &N){
	Sample sample;
	double s, t;
	s = rand(0.0, 1.0);
	t = rand(0.0, 1.0);
	Vec3 localNormal = {0,0,1}; 
	float x = (sqrt(t) * cos(2 * Pi*s));
	float y = (sqrt(t) * sin(2 * Pi*s));
	float z = sqrt(1-(x*x)-(y*y));
	Vec3 xyzVector = { x, y, z };
	Vec3 mirror = Unit(localNormal + N); //Vector "espejo" medio
	sample.P = Unit(Reflection(-xyzVector, mirror));
	sample.w = Pi;
	return sample;
}

//Devuelve una muestra en el lóbulo especular. Este es un tipo de muestreo por importancia
Sample Raytracer::SampleSpecularLobe(const Vec3 &R, float phong_exp){
	Sample sample;
	double s, t;
	s = rand(0.0, 1.0);
	t = rand(0.0, 1.0);
	Vec3 localNormal = { 0, 0, 1 };	
	float x = sqrt(1 - pow(t, 2 / (phong_exp + 1)))*cos(2 * Pi*s);
	float y = sqrt(1 - pow(t, 2 / (phong_exp + 1)))*sin(2 * Pi*s);
	float z = sqrt(1 - (x*x) - (y*y));
	Vec3 xyzVector = { x, y, z };
	Vec3 mirror = Unit(localNormal + R);//Vector "espejo" medio
	sample.P = Unit(Reflection(-xyzVector, mirror));
	sample.w = (2 * Pi / (phong_exp + 2));
	return sample;
}
