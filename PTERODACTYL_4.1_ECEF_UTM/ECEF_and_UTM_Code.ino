//The code in this Tab plus the section in the main 'PTERODACTYL_ECEF_UTM' labled GPS variables allow for the 

// %%%%%%%%%%%%%%% BELOW IS THE SECTION FOR RECEIVING NAV CLASS GPS DATA  %%%%%%%%%%%%%%%%%%%%%%
//function to set up the sparkFunGNSS object
void ecefUbloxSetup() {
  //serial connection should be begun already
//  sparkFunGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  Serial.println("establishing connection For ecef GPS");

  while (!sparkFunGNSS.begin(ecefSerial))
  {
    if (usingM9N == true) {
      Serial.println("The variable usingM9N == true so trying to connect to M9N at 38400 baudrate\nIF USING M8N -> change this variable to false under the first tab line 10.\nM8N req 9600, M9N req 38400");
      Serial.println("Trying 38400 baud");
      ecefSerial.begin(38400);
      delay(500);
    } else {
      Serial.println("The variable usingM9N == false so trying to connect to M8N at 9600 baudrate\nIF USING M9N -> change this variable to true under the first tab line 10.\nM8N req 9600, M9N req 38400");
      Serial.println("Trying 9600 baud");
      ecefSerial.begin(9600);
      delay(500);
    }
  }
  if (usingM9N) {
    Serial.println("sparkFunGNSS connected at 38400 baud");
  } else {
    Serial.println("sparkFunGNSS connected at 9600 baud");
  }
  Serial.println("Setting Nav frequency");
  sparkFunGNSS.setNavigationFrequency(5); //Produce 5 solutions per second [set number of times the callback function in called per second]
  //setting the auto callback for the ECEF data
  sparkFunGNSS.setAutoNAVPOSECEFcallbackPtr(&ecefDataUpdate);
  Serial.println("Return of satAutoNavPOSECEF(): " + String(sparkFunGNSS.setAutoNAVPOSECEFcallbackPtr(&ecefDataUpdate)) + ". It should be a one (1) if successful");
 // Enable automatic NAV ECEF messages with callback to printECEFdata
}

//checks for new data, updates GPS and then if new ECEF data is available then funs 'ecefDataUpdate()'
void updateECEF() {
  sparkFunGNSS.checkUblox(); // Check for the arrival of new data and process it.
  sparkFunGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed. (callback function is update ECEF)
}

//funciton gets called any time there is updated ecef data
void ecefDataUpdate(UBX_NAV_POSECEF_data_t *ubxDataStruct)
{
  //Serial.println("ecefeDateUpdate Has been called");
  ecefX = ubxDataStruct->ecefX; // Print the X Value (output is usually in cm but converting to meters fro calcultions)
  ecefX /= 100;
  ecefY = ubxDataStruct->ecefY; // Print the Y value
  ecefY /= 100;
  ecefZ = ubxDataStruct->ecefZ; // Print the Z value
  ecefZ /= 100;
  posAcc = ubxDataStruct->pAcc; // Print the height above mean sea level
  posAcc /= 100;
  SIV = sparkFunGNSS.getSIV();
  //calculate
  ecefToLLA(ecefX, ecefY, ecefZ, latCalc, longCalc, altCalc);
  //return;
}

//funciton to update the UTM coordinates.
void updateUTM() {
  //call function to find the UTM variables. the lat and long variabels that are being passed in are taken directally from the PVT values
  LLtoUTM(latitudeGPS, longitudeGPS, UTMNorthing, UTMEasting, UTMZoneNum); //conversion is using direct LLA values
  UTMZoneLetter = UTMLetterDesignator(latitudeGPS); //passes in current latitude in order to find the letter designation for northing and easting
}
// %%%%%%%%%%%%%% ALOVE IS THE SECTION FOR RECEIVING NAV CLASS GPS DATA  %%%%%%%%%%%%%%%%%%%%%%%%%%%




//%%%%%%%%%%%%%%%%%%%%%%%  BELOW  are functions to convert ECEF to LLA format %%%%%%%%%%%%%%%%%%%%%%%

#define pi 3.1415926535897
#define rad2Deg 180.00/pi

const double a = 6378137.00;
const double b = 6356752.31424518;
const double f = 1 / (298.257223563);
const double e = sqrt( (pow(a, 2) - pow(b, 2)) / pow(a, 2) );
const double ePrime = sqrt( (pow(a, 2) - pow(b, 2)) / pow(b, 2) );


//below function takes what ever ecef values are stored at time of calling and uses them to convert into LLA format
// x, y, and z are their respective ecef coordinates, and the lat, long, and alt should be the variables the calculations are saved to
void ecefToLLA(double x, double y, double  z, double  &latitude, double  &longitude, double  &alt)
{
  //finding auxillary values to simplify formulas:
  double p = sqrt(pow(x, 2) + pow(y, 2));
  double theta = atan((z * a) / (p * b));

  //converting into long
  longitude = atan(y / x) - pi;
  longitude = longitude * 180.0 / pi;

  //converting into latitude
  double latNumerator = z + pow(ePrime, 2) * b * pow(sin(theta), 3);
  double latDenominator = p - pow(e, 2) * a * pow(cos(theta), 3);
  latitude = atan(latNumerator / latDenominator) * (180.0 / pi);


  //finding radius of cervature 'N' in order to calculator altitude:
  double n = a / (sqrt( 1 - ( pow(e, 2) * pow(sin(latitude * pi / 180.0) , 2))));
  //final calculation for the altitude
  alt = (p / cos(latitude * pi / 180.0)) - n;

  //convert to ft
  alt *= 3.28;

  return;
}
//%%%%%%%%%%%%%%%%%%%%%%% ABOVE  are functions to convert ECEF to LLA format %%%%%%%%%%%%%%%%%%%%%%%


//%%%%%%%%%%%%%%%%%%% BELOW ARE FUNCTIONS TO CONVERT FROM LLA TO UTM COORDINATES %%%%%%%%%%%%%%%%%%%
//this is a function that will convert a given LLA Value to UTM (Easting and Northing) coordinates.
// taken from:  https://github.com/bakercp/ofxGeo/blob/master/libs/UTM/include/UTM/UTM.h
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
// Grid granularity for rounding UTM coordinates to generate MapXY.
const double grid_size = 100000.0;    ///< 100 km grid

// WGS84 Parameters
#define WGS84_A    6378137.0   ///< major axis
#define WGS84_B   6356752.31424518  ///< minor axis
#define WGS84_F   0.0033528107    ///< ellipsoid flattening
#define WGS84_E   0.0818191908    ///< first eccentricity
#define WGS84_EP  0.0820944379    ///< second eccentricity

// UTM Parameters
#define UTM_K0    0.9996      ///< scale factor
#define UTM_FE    500000.0    ///< false easting
#define UTM_FN_N  0.0           ///< false northing, northern hemisphere
#define UTM_FN_S  10000000.0    ///< false northing, southern hemisphere
#define UTM_E2    (WGS84_E*WGS84_E) ///< e^2
#define UTM_E4    (UTM_E2*UTM_E2)   ///< e^4
#define UTM_E6    (UTM_E4*UTM_E2)   ///< e^6
#define UTM_EP2   (UTM_E2/(1-UTM_E2)) ///< e'^2

/**
   Determine the correct UTM letter designator for the
   given latitude

   @returns 'Z' if latitude is outside the UTM limits of 84N to 80S

   Written by Chuck Gantz- chuck.gantz@globalstar.com
*/
static inline char UTMLetterDesignator(double Lat)
{
  char LetterDesignator;

  if     ((84 >= Lat) && (Lat >= 72))  LetterDesignator = 'X';
  else if ((72 > Lat) && (Lat >= 64))  LetterDesignator = 'W';
  else if ((64 > Lat) && (Lat >= 56))  LetterDesignator = 'V';
  else if ((56 > Lat) && (Lat >= 48))  LetterDesignator = 'U';
  else if ((48 > Lat) && (Lat >= 40))  LetterDesignator = 'T';
  else if ((40 > Lat) && (Lat >= 32))  LetterDesignator = 'S';
  else if ((32 > Lat) && (Lat >= 24))  LetterDesignator = 'R';
  else if ((24 > Lat) && (Lat >= 16))  LetterDesignator = 'Q';
  else if ((16 > Lat) && (Lat >= 8))   LetterDesignator = 'P';
  else if (( 8 > Lat) && (Lat >= 0))   LetterDesignator = 'N';
  else if (( 0 > Lat) && (Lat >= -8))  LetterDesignator = 'M';
  else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
  else if ((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
  else if ((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
  else if ((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
  else if ((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
  else if ((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
  else if ((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
  else if ((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
  else if ((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
  // 'Z' is an error flag, the Latitude is outside the UTM limits
  else LetterDesignator = 'Z';
  return LetterDesignator;
}

/**
   Convert lat/long to UTM coords.  Equations from USGS Bulletin 1532

   East Longitudes are positive, West longitudes are negative.
   North latitudes are positive, South latitudes are negative
   Lat and Long are in fractional degrees

   Written by Chuck Gantz- chuck.gantz@globalstar.com
*/
static inline void LLtoUTM(const double Lat, const double Long,
                           double &UTMNorthing, double &UTMEasting,
                           int &UTMZone) //this has been edited so it includes UTM as an int not a char
{
  double a = WGS84_A;
  double eccSquared = UTM_E2;
  double k0 = UTM_K0;

  double LongOrigin;
  double eccPrimeSquared;
  double N, T, C, A, M;

  //Make sure the longitude is between -180.00 .. 179.9
  double LongTemp = (Long + 180) - int((Long + 180) / 360) * 360 - 180;
  // Serial.println("HERE IS LONG TEMP: " + String(LongTemp));

  double LatRad = Lat * DEG_TO_RAD;
  double LongRad = LongTemp * DEG_TO_RAD;
  double LongOriginRad;
  int    ZoneNumber;

  ZoneNumber = int((LongTemp + 180) / 6) + 1;

  if ( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
    ZoneNumber = 32;

  // Special zones for Svalbard
  if ( Lat >= 72.0 && Lat < 84.0 )
  {
    if (      LongTemp >= 0.0  && LongTemp <  9.0 ) ZoneNumber = 31;
    else if ( LongTemp >= 9.0  && LongTemp < 21.0 ) ZoneNumber = 33;
    else if ( LongTemp >= 21.0 && LongTemp < 33.0 ) ZoneNumber = 35;
    else if ( LongTemp >= 33.0 && LongTemp < 42.0 ) ZoneNumber = 37;
  }
  // +3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;
  LongOriginRad = LongOrigin * DEG_TO_RAD;

  //compute the UTM Zone from the latitude and longitude
  // sprintf(UTMZone, "%d%c", ZoneNumber, UTMLetterDesignator(Lat)); //commented out bc doesnt work

  UTMZone = ZoneNumber;    //save the found zone number to UTMZone

  eccPrimeSquared = (eccSquared) / (1 - eccSquared);

  N = a / sqrt(1 - eccSquared * sin(LatRad) * sin(LatRad));
  T = tan(LatRad) * tan(LatRad);
  C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
  A = cos(LatRad) * (LongRad - LongOriginRad);

  M = a * ((1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64
            - 5 * eccSquared * eccSquared * eccSquared / 256) * LatRad
           - (3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32
              + 45 * eccSquared * eccSquared * eccSquared / 1024) * sin(2 * LatRad)
           + (15 * eccSquared * eccSquared / 256
              + 45 * eccSquared * eccSquared * eccSquared / 1024) * sin(4 * LatRad)
           - (35 * eccSquared * eccSquared * eccSquared / 3072) * sin(6 * LatRad));

  UTMEasting = (double)
               (k0 * N * (A + (1 - T + C) * A * A * A / 6
                          + (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A * A * A * A * A / 120)
                + 500000.0);

  UTMNorthing = (double)
                (k0 * (M + N * tan(LatRad)
                       * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24
                          + (61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared) * A * A * A * A * A * A / 720)));

  if (Lat < 0)
  {
    //10000000 meter offset for southern hemisphere
    UTMNorthing += 10000000.0;
  }
}

/**
   Converts UTM coords to lat/long.  Equations from USGS Bulletin 1532

   East Longitudes are positive, West longitudes are negative.
   North latitudes are positive, South latitudes are negative
   Lat and Long are in fractional degrees.

   Written by Chuck Gantz- chuck.gantz@globalstar.com
*/
static inline void UTMtoLL(const double UTMNorthing, const double UTMEasting,
                           const char* UTMZone, double& Lat,  double& Long )
{
  double k0 = UTM_K0;
  double a = WGS84_A;
  double eccSquared = UTM_E2;
  double eccPrimeSquared;
  double e1 = (1 - sqrt(1 - eccSquared)) / (1 + sqrt(1 - eccSquared));
  double N1, T1, C1, R1, D, M;
  double LongOrigin;
  double mu, phi1Rad;
  double x, y;
  int ZoneNumber;
  char* ZoneLetter;

  x = UTMEasting - 500000.0; //remove 500,000 meter offset for longitude
  y = UTMNorthing;

  ZoneNumber = strtoul(UTMZone, &ZoneLetter, 10);
  if ((*ZoneLetter - 'N') < 0)
  {
    //remove 10,000,000 meter offset used for southern hemisphere
    y -= 10000000.0;
  }

  //+3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;
  eccPrimeSquared = (eccSquared) / (1 - eccSquared);

  M = y / k0;
  mu = M / (a * (1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64
                 - 5 * eccSquared * eccSquared * eccSquared / 256));

  phi1Rad = mu + ((3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * mu)
                  + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * mu)
                  + (151 * e1 * e1 * e1 / 96) * sin(6 * mu));

  N1 = a / sqrt(1 - eccSquared * sin(phi1Rad) * sin(phi1Rad));
  T1 = tan(phi1Rad) * tan(phi1Rad);
  C1 = eccPrimeSquared * cos(phi1Rad) * cos(phi1Rad);
  R1 = a * (1 - eccSquared) / pow(1 - eccSquared * sin(phi1Rad) * sin(phi1Rad), 1.5);
  D = x / (N1 * k0);

  Lat = phi1Rad - ((N1 * tan(phi1Rad) / R1)
                   * (D * D / 2
                      - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * eccPrimeSquared) * D * D * D * D / 24
                      + (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * eccPrimeSquared
                         - 3 * C1 * C1) * D * D * D * D * D * D / 720));

  Lat = Lat * RAD_TO_DEG;

  Long = ((D - (1 + 2 * T1 + C1) * D * D * D / 6
           + (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * eccPrimeSquared + 24 * T1 * T1)
           * D * D * D * D * D / 120)
          / cos(phi1Rad));
  Long = LongOrigin + Long * RAD_TO_DEG;

}
//%%%%%%%%%%%%%%%%%%% ABOVE ARE FUNCTIONS TO CONVERT FROM LLA TO UTM COORDINATES %%%%%%%%%%%%%%%%%%%
