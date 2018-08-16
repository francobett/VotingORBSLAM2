
/*
 *
 *  New Thread for Voting Scheme
 *  Author: Betetto FRanco
 */
#include "Voting.h"
#include <iostream>
#include <fstream>
#include <string>
#include <mutex>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <limits>
#include <stdexcept>
#include"ORBmatcher.h"
#include"Frame.h"

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ml.hpp>

//#include <boost/filesystem.hpp>
//#include <boost/algorithm/string.hpp>
//#include <boost/lexical_cast.hpp>

using namespace std;
using namespace cv;
using namespace xfeatures2d;
//using namespace boost::filesystem;
//using namespace boost;

namespace ORB_SLAM2
{
Voting::Voting():
		// Initialize flags
		mbResetRequested(false), mbFinishRequested(false), mbFinished(true),
	    mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
}

// Set's for another Threads
void Voting::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Voting::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void Voting::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

// Run

void Voting::Run()
{

	mbFinished =false;

	cout << "Voting Scheme Loaded!" << endl;

	int contador = 1;

	while(1)
	{
		// LocalMapping will see that Voting is busy
		SetAcceptKeyFrames(false);

		// Check if there are keyframes in the queue
		if(CheckNewKeyFrames())
		{

			ProcessNewKeyFrame();

			cout << endl << endl << "KeyFrame detected on Voting: " << contador << " ID KF: "<< mpCurrentKeyFrame -> mnId << endl;
			contador += 1;

			ClassifyKeyFrame();

			// Insert current KeyFrame to Loop Closing thread
			mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
		}
		else if(Stop())
		{
			// Safe area to stop
			while(isStopped() && !CheckFinish())
			{
				usleep(3000);
			}
			if(CheckFinish())
				break;
		}

		ResetIfRequested();

		// Local Mapping will see that Voting is free
		SetAcceptKeyFrames(true);

		if(CheckFinish())
			break;

		usleep(3000);
	}
	cout << "Se procesaron " << contador << "key frames" << endl;
	SetFinish();



}

void Voting::ClassifyKeyFrame()
{
	cout << "El keyframe tiene keypoints: " << mpCurrentKeyFrameKeyPoints.size()  << endl;

	// Variables to use

	// Key Points of the current KeyFrame
	mpCurrentKeyFrameKeyPoints = mpCurrentKeyFrame -> mvKeys;

	// Map Points of the current KeyFrame
	mpCurrentKeyFrameMapPoints = mpCurrentKeyFrame -> mvpMapPoints;

	// Image of the keyframe
	cv::Mat img = mpCurrentKeyFrame -> imGray ;

	// xWindow is the width (ancho) of the patch
	int xWindow = 300;

	// yWindow is the height (alto) of the patch
	int yWindow = 300;

	// despX is the displacement in X to be used
	int despX = xWindow * 1;

	// despY is the displacement in Y to be used
	int despY = yWindow * 1;

	// widthImg is the width (ancho) of the image
	int widthImg = 720;

	// heightImg is the height (alto) of the image
	int heightImg = 1280;

	// route of the .csv generated
	char* rutaOut = "/home/beto/ORB-SLAM/Clasificator/";

	// .R file
	char* rFile = "/home/beto/ORB-SLAM/ORB_SLAM2/patchClass.R";

	// Threshold Voting
	double thresholdVoting = 0.5;

	// Vector that has the start of the patches of the image
	std::vector<Point2f> initPatches;


	// TODO Verificar que los patches esten bien ya que siempre se exceden 5 que no tienen kpoints
	// Set the upper left corner of the patches
	for(int j=0;j<heightImg/despY;j++){ // Divide by patches in height
		for(int i=0; i<widthImg/despX ;i++){ // Divide by patches in width
			// Coords of the current patch
			int startX = i*despX;
			int startY = j*despY;
			int endX = startX + xWindow;
			int endY = startY + yWindow;

			// If it exceeds the width or height
			if(endX >= widthImg) {endX = widthImg-1; startX = endX-xWindow;}
			if(endY >= heightImg) {endY = heightImg-1; startY = endY-yWindow;}

			// Add the upper left corner point of the patch to vector
			Point2f initPatch = Point2f(startX,startY);
			initPatches.push_back(initPatch);
		}
	}


	// Calculate keypoints and descriptors
	Ptr<SIFT> edetector = xfeatures2d::SIFT::create(); // ****
	//Ptr<FeatureDetector> edetector = ORB::create();
	//vector<Mat> descriptorImages;
	vector<Mat> keypointsCount;

	//Var for KeyPoints of the image
	vector<KeyPoint> keypointsImg;

	// Set the keypoints with the detector
	edetector -> detect( img, keypointsImg );  // ****

	cout << "SIFT has detected: "<< keypointsImg.size() << " keypoints in this keyframe" << endl << endl;
	//Mat for the descriptor of the image
	Mat descriptor;


	keypointsCount.push_back(descriptor);


	edetector->compute(img, keypointsImg, descriptor);

	//New matrix of Nx8 where N is the amount of keypoints
	Mat coordKP(descriptor.rows, 8, descriptor.type());


	// For each row, set the coords
	for (int i = 0; i < coordKP.rows; i++) {
		coordKP.at<float>(i, 0) = keypointsImg[i].pt.x;
		coordKP.at<float>(i, 1) = keypointsImg[i].pt.y;
		// This column is for saving the landmark en el flannmatcher
		coordKP.at<float>(i,2)    = -1.0;
		coordKP.at<float>(i, 3) = keypointsImg[i].angle;
		coordKP.at<float>(i, 4) = keypointsImg[i].class_id;
		coordKP.at<float>(i, 5) = keypointsImg[i].octave;
		coordKP.at<float>(i, 6) = keypointsImg[i].response;
		coordKP.at<float>(i, 7) = keypointsImg[i].size;
	}




	/* Concat coordKP and descriptor, and the result its in descriptor
	* rows[coordKP] = rows[descriptor]
	 *columns[descriptor] (after hconcat) = columns[descriptor] + columns[coordKP]*/
	hconcat(coordKP, descriptor, descriptor);

	// Add matrix descriptor, to the vector of descriptors
	//descriptorImages.push_back(descriptor);



	// Matrix with info about coords X,Y
	Mat coor = descriptor.colRange(0,2).clone();
	// Matrix with info about descriptor
	Mat desc = descriptor.colRange(8, descriptor.cols).clone();

	// For each patch of the current image
	for(int i =0; i<initPatches.size(); i++){
		// Matrix where each row has a descriptor of the KeyPoint that is inside of the patch
		Mat descriptorsPatch;

		// Matrix with the coords of each keypoint, first row with the X and the second with the Y
		Mat coordsPatch;

		// For each keypoint in current keyframe, verify if is inside the patch
		for(int j = 0; j < descriptor.rows; j ++){

			if( descriptor.at<float>(j,0) > initPatches[i].x && // X  > X  of the start of the patch
				descriptor.at<float>(j,0) < initPatches[i].x + xWindow && // X  <  X  of the end of the patch
				descriptor.at<float>(j,1)  > initPatches[i].y && // Y  >  Y  of the start of the patch
				descriptor.at<float>(j,1)  < initPatches[i].y+yWindow){ // Y  <  Y  of the end of the patch

				//Add descriptor of that keypoint to current patch
				descriptorsPatch.push_back(desc.row(j));

				//Add coords of keypoint to coordsPatch
				coordsPatch.push_back(coor.row(j));
			}
		}


		//Classification
		string modelsPath = "/home/beto/ORB-SLAM/Clasificator/modelos/";
		int dictSize = 25;
		int iter = 1;
		// Call PatchClass function
		bool pred = this -> PatchClass(descriptorsPatch,modelsPath,dictSize,iter, rutaOut, rFile);

		// Verify the keypoints of keyframe inside of the current patch

		// For each Keypoints of keyframe (orb)
		for(unsigned j = 0; j <= mpCurrentKeyFrameKeyPoints.size(); j ++){

			// If its inside the patch
			if( mpCurrentKeyFrameKeyPoints[j].pt.x > initPatches[i].x && // X  > X  of the start of the patch
				mpCurrentKeyFrameKeyPoints[j].pt.x < initPatches[i].x + xWindow && // X  <  X  of the end of the patch
				mpCurrentKeyFrameKeyPoints[j].pt.y > initPatches[i].y && // Y  >  Y  of the start of the patch
				mpCurrentKeyFrameKeyPoints[j].pt.y < initPatches[i].y+yWindow){ // Y  <  Y  of the end of the patch



				// If the position J has a mapPoint and this its not bad
				if( mpCurrentKeyFrameMapPoints[j] != NULL && !(mpCurrentKeyFrameMapPoints[j] -> isBad())){
					// Get total votes and positive votes from MapPoint J
					int &totalVotes = mpCurrentKeyFrameMapPoints[j] -> totalVotes;
					int &positiveVotes = mpCurrentKeyFrameMapPoints[j] -> positiveVotes;


					// Add one general vote
					totalVotes += 1;

					// If its true, add a positive vote
					if(pred){
						positiveVotes += 1;
					}

				}

			}

		}

		/*
		cout << "Los keypoints de ORB en el patch son "<< keypointInPatch.size() << endl;


		// If patch its 'yema',
		if (pred){
			cout << "Patch yema " << endl;
		}else{
			cout << "Patch NO yema " << endl;
		}
		*/

	}


}

bool Voting::PatchClass(cv::Mat SIFTdescriptors, string modelsPath, int dictSize, int iter, char* salida, char* rFile)
{
	cout<< "-------------------------------------------------" << endl;
	// NO hay descriptores, devuelve false
	if (SIFTdescriptors.empty()){
		//cout<< "-------------------------------------------------" << endl;
		cout << "No keypoints in this patch" << endl;
		cout<< "-------------------------------------------------" << endl;
		return false;
	}


	cout << "The patch has: "<< SIFTdescriptors.rows <<" keypoints" << endl ;
	//To store the dictionary
	Mat dictionary;
	//Read the stored dictionary

	//prepare BOW descriptor extractor from the dictionary
	//to store the input file names
	char * filedic = new char[100];
	//create the file name of an image
	sprintf(filedic, "%sdict-s%d.yml", modelsPath.c_str(), dictSize);

	FileStorage fs(filedic, FileStorage::READ);
	fs["vocabulary"] >> dictionary;
	fs.release();

	// End read stored dictionary

	//To store the BoF representation of the image
	Mat bowDescriptor;


	//create a nearest neighbor matcher and SIFT feature descriptor
	Ptr<DescriptorMatcher> matcher(new FlannBasedMatcher);


	//Create Sift feature point extracter
	cv::Ptr<xfeatures2d::SIFT> detector = xfeatures2d::SIFT::create();
	//Ptr<DescriptorExtractor> extractor = ORB::create();

	//create BoF descriptor extractor
	BOWImgDescriptorExtractor bowDE(detector, matcher);
	//Set the dictionary with the vocabulary we created in the first step
	bowDE.setVocabulary(dictionary);
	//extract BoF descriptor from given image
	bowDE.compute(SIFTdescriptors, bowDescriptor);

	bool pred = false;
	//BoF descriptors classification
	if (!(bowDescriptor.empty())) {
		//To store the descriptor file name
		char * filecsv = new char[200];
		//the descriptor file with the location.
		sprintf(filecsv, "%shist-s25-it1.csv",salida);
		//open the file to write the resultant descriptor
		std::ofstream fs1(filecsv);
		//image's info
		fs1 << format(bowDescriptor, Formatter::FMT_CSV)<< endl;
		fs1.close();

		char* r_cmd = new char[10000];
		sprintf(r_cmd, "R -q -e  \"source('%s');patchClassifier('%s', '%s', %d, %d)\"",rFile,
						modelsPath.c_str(), filecsv, dictSize, iter);
		const char* com = r_cmd;
		string result = "";
		// Result
		char buffer[128];

		/* Open the R file with the value of r_cmd (com). It has 2 parts
		 * Where the R file is
		 *  - source(rFile)
		 * Params for patchClassifier function
		 * - patchClassifier(models, file with descriptors, size of dictionary, iteration)
		 */

		FILE* pipe = popen(com, "r");
		if (!pipe) throw std::runtime_error("popen() failed!");
		try {
			while (!feof(pipe)) {
				if (fgets(buffer, 128, pipe) != NULL)
					result += buffer;
			}
		} catch (...) {
			pclose(pipe);
			throw;
		}
		pclose(pipe);

		/* The result has 3 parts:
		 * - What goes into the function in r and the source of this
		 * - The probabilities if that patch its bud (yema) or not
		 * - And the probability to be bud
		*/
		//cout << result <<endl;

		// Get the probability that it is bud and set in result
		size_t found = result.find_last_of("]");
		result = result.substr(found+2,string::npos);
		cout << "Probabilidades de Yema: " << result << endl;
		// If the result is more than  minValue, the prediction of the classifier its consider true
		double minValue = 0.5;
		(atof(result.c_str())>minValue)?pred=true:pred=false;


		cout<< "Classificator result: "<< (pred? "true" : "false")<< endl;

		// Command to remove the filsecsv generated
		char* comando = new char[510];
		sprintf(comando, "rm %s", filecsv);
		std::system(comando);
	} else
		cout << "Error al calcular el BoF descriptor\n";
	return pred;

}

// A new KF is inserted in the Voting Queue by LocalMapping

void Voting::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexVotingQueue);
    //if(pKF->mnId!=0)
    mlKeyFramesVotingQueue.push_back(pKF);
}

// This function allows that Voting can receibe new KF

void Voting::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

// This function verify if there is some KF in the Voting Queue

bool Voting::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexVotingQueue);
    return(!mlKeyFramesVotingQueue.empty());
}

/* This function catchs the first KF on the VotingQueue to be assign as the CurrentKeyFrame
 * If the KF is bad it is ignored and the following is extracted until it isnt bad
 */

void Voting::ProcessNewKeyFrame()
{
	do
    {
        unique_lock<mutex> lock(mMutexVotingQueue);
        mpCurrentKeyFrame = mlKeyFramesVotingQueue.front();
        mlKeyFramesVotingQueue.pop_front();
    }
	while(mpCurrentKeyFrame -> isBad());
}

// This function allow to stop this thread

bool Voting::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Voting STOP" << endl;
        return true;
    }

    return false;
}

// Verify if Voting thread is stopped

bool Voting::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

// If the reset is requested, reset the Voting Queue

void Voting::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
    	mlKeyFramesVotingQueue.clear();
        //mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
    }
}

// Verify if this thread is finished

bool Voting::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

// Set the flags Finished and Stopped in true

void Voting::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

}//namespace ORB_SLAM

