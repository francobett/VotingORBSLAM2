
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

	int contador = 0;

	while(1)
	{
		// LocalMapping will see that Voting is busy
		SetAcceptKeyFrames(false);

		// Check if there are keyframes in the queue
		if(CheckNewKeyFrames())
		{

			ProcessNewKeyFrame();

			cout << "KeyFrame detected on Voting: " << contador << endl;
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

		// Local Mapping will see that Voting is busy
		SetAcceptKeyFrames(true);

		if(CheckFinish())
			break;

		usleep(3000);
	}

	SetFinish();



}

void Voting::ClassifyKeyFrame()
{
	/*TODO
	 * Implementación EJECUTAR ALGORITMO DE CLASIFICACIÓN SOBRE KeyFrameActual
	 * Y ESTABLECER QUE KeyPoints PERTENECEN A PATCHES CLASIFICADOS COMO YEMA.
	*/


	cout << "El keyframe tiene keypoints: " << mpCurrentKeyFrameKeyPoints.size()  << endl;

	// Variables to use

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

	// .R file TODO
	char* rFile = "./patchClass.r";

	// Vector that has the start of the patches of the image
	std::vector<Point2f> initPatches;

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

	// Get the keypoints of the image
	cv::Mat img = mpCurrentKeyFrame -> imGray ;

	// Get KeyPoints of current keyframe
	mpCurrentKeyFrameKeyPoints = mpCurrentKeyFrame -> mvKeys;

	// Get Descriptors of current keyframe
	mCurrentKeyFrameDescriptors = mpCurrentKeyFrame -> mDescriptors;

	// For each patch of the current image
	for(unsigned i =0; i<=initPatches.size(); i++){
		// Matrix where each row has a descriptor of the KeyPoint that is inside of the patch
		Mat descriptorsPatch;

		// Matrix with the coords of each keypoint, first row with the X and the second with the Y
		Mat coordsPatch;

		// For each keypoint in current keyframe, verify if is inside the patch
		for(unsigned j = 0; j <= mpCurrentKeyFrameKeyPoints.size(); j ++){

			if( mpCurrentKeyFrameKeyPoints[j].pt.x > initPatches[i].x && // X  > X  of the start of the patch
				mpCurrentKeyFrameKeyPoints[j].pt.x < initPatches[i].x + xWindow && // X  <  X  of the end of the patch
				mpCurrentKeyFrameKeyPoints[j].pt.y > initPatches[i].y && // Y  >  Y  of the start of the patch
				mpCurrentKeyFrameKeyPoints[j].pt.y < initPatches[i].y+yWindow){ // Y  <  Y  of the end of the patch

				//Add descriptor of that keypoint to current patch
				descriptorsPatch.push_back(mCurrentKeyFrameDescriptors.row(j));

				//Add coords of keypoint to coordsPatch
				//Mat coordsKeyPoint = Mat(mpCurrentKeyFrameKeyPoints[j].pt.x,mpCurrentKeyFrameKeyPoints[j].pt.y);
				//coordsPatch.push_back(coordsKeyPoint);
			}
		}


		//Classification
		string modelsPath = "../Clasificator/modelos/";
		int dictSize = 25;
		int iter = 1;
		// Call PatchClass function
		bool pred = this -> PatchClass(descriptorsPatch,modelsPath,dictSize,iter, rFile);



		// If patch its 'yema',
		if (pred){
			cout << "Patch yema " << endl;
			for( int j=0; j<coordsPatch.rows;j++){
				//keypointsCount[image].push_back(coordenadasPatch.row(j));

			}
		}else{
			cout << "Patch NO yema " << endl;
		}

	}

	// Get MapPoints of localmap of current keyframe associated to keypoints
	//mpCurrentKeyFrameMapPoints = mpCurrentKeyFrame -> GetMapPoints();

	//cout << "N° MapPoints Matches: " << mpCurrentKeyFrameMapPoints.size()  << endl << endl;

	// Verify if mappoints have are in the current key frame (it should be true for all mappoints)
	/*for (auto mp: mpCurrentKeyFrameMapPoints){
		if ( !(mp -> IsInKeyFrame(mpCurrentKeyFrame) )){
			cout << "No correspondence"  << endl;
		}
	}
	*/

}

bool Voting::PatchClass(cv::Mat SIFTdescriptors, string modelsPath, int dictSize, int iter, char* rFile)
{
	// NO hay descriptores, devuelve false
	if (SIFTdescriptors.empty()){
		cout << "No descriptors in this patch" << endl;
		return false;
	}

	cout << "en este patch cayeron "<<SIFTdescriptors.rows<<" keypoints \n";
	//To store the dictionary
	Mat dictionary;
	//Read the stored dictionary

	//prepare BOW descriptor extractor from the dictionary
	//to store the input file names
	char * filedic = new char[100];
	//create the file name of an image
	sprintf(filedic, "%sdict-s%d.yml", modelsPath.c_str(), dictSize);
	cout << "Path del diccionario: " << filedic << endl ;
	FileStorage fs(filedic, FileStorage::READ);
	fs["vocabulary"] >> dictionary;
	fs.release();
	//To store the BoF representation of the image
	Mat bowDescriptor;
	//create a nearest neighbor matcher and SIFT feature descriptor
	cout << "bowDEscriptor creado "<< bowDescriptor  << endl;

	//cv::Ptr<xfeatures2d::SIFT> detector = xfeatures2d::SIFT::create();

	//Ptr<DescriptorMatcher> matcher(new FlannBasedMatcher);
	//cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased");

	//create a nearest neighbor matcher
	Ptr<DescriptorMatcher> matcher(new FlannBasedMatcher);
	cout << "Matcher creado";
	//create Sift feature point extracter
	cv::Ptr<xfeatures2d::SIFT> detector = xfeatures2d::SIFT::create();
	cout << "Detector creado";

	//create BoF descriptor extractor
	BOWImgDescriptorExtractor bowDE(detector, matcher);
	//Set the dictionary with the vocabulary we created in the first step
	bowDE.setVocabulary(dictionary);
	//extract BoF descriptor from given image
	bowDE.compute(SIFTdescriptors, bowDescriptor);

	bool pred = false;
	//BoF descriptors classification
	if (!(bowDescriptor.empty())) {
		cout << "Bowdescriptor con algun elemento";
		//To store the descriptor file name
		char * filecsv = new char[200];
		//the descriptor file with the location.
		//sprintf(filecsv, "%s/hist-s25-it1.csv",salida);
		//open the file to write the resultant descriptor
		std::ofstream fs1(filecsv);
		//image's info
		fs1 << format(bowDescriptor, Formatter::FMT_CSV)<< endl;
		fs1.close();

		char* r_cmd = new char[10000];
		sprintf(r_cmd, "R -q -e  \"source('%s');patchClassifier('%s', '%s', %d, %d)\"",rFile,
						modelsPath.c_str(), filecsv, dictSize, iter);
		const char* com = r_cmd;
		string result;
		// Result
		char buffer[128];

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

		cout << result <<endl;
		size_t found = result.find_last_of("]");
		result = result.substr(found+2,string::npos);
		(atof(result.c_str())>0.5)?pred=true:pred=false;
		cout<< "resultado del clasificador "<<(pred? " true" : " false")<<endl;
		char* comando = new char[510];
		sprintf(comando, "rm %s", filecsv);
		std::system(comando);
	} else
		cout << "Error al calcular el BoF descriptor\n";
		//todo umbral de clasificación
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

// This function catchs the first KF on the VotingQueue to be assign as the CurrentKeyFrame

void Voting::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexVotingQueue);
        mpCurrentKeyFrame = mlKeyFramesVotingQueue.front();
        mlKeyFramesVotingQueue.pop_front();
    }
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

