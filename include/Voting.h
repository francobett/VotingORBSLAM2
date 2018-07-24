/*
 * Voting.h
 *
 *  Created on: 27 jun. 2018
 *      Author: beto
 */

#ifndef INCLUDE_VOTING_H_
#define INCLUDE_VOTING_H_

#include "LoopClosing.h"
#include "LocalMapping.h"
#include "Tracking.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include <mutex>

namespace ORB_SLAM2
{

// TODO importar clases a utilizar por este hilo
class Tracking;
class LocalMapping;
class LoopClosing;

class Voting {

public:
	Voting();

	// Set another threads

	void SetLoopCloser(LoopClosing* pLoopCloser);

	void SetTracker(Tracking* pTracker);

	void SetLocalMapper(LocalMapping* pLocalMapper);;

	// Main Function
	void Run();

	// Runs a classification algorithm to know what KeyPoints on KeyFrame are bud
	void ClassifyKeyFrame();

	// Runs the classifier of patches
	bool PatchClass(cv::Mat descriptors, string modelsPath, int dictSize, int iter, char* rFile) ;

	// Function that is called by LocalMapping for insert the new keyframe from that thread to Voting thread
	void InsertKeyFrame(KeyFrame *pKF);

	// Thread Synch
	bool Stop();
    bool isStopped();
	void SetAcceptKeyFrames(bool flag);

	// To count the KF in the queue of voting

	int KeyframesInQueue(){
	        unique_lock<std::mutex> lock(mMutexVotingQueue);
	        return mlKeyFramesVotingQueue.size();
	    }

protected:
	// Verify if there is some KF in the queue
	bool CheckNewKeyFrames();

	// Process the KF to work with it
	void ProcessNewKeyFrame();

	// Mutex for KeyFrames queue
	std::mutex mMutexVotingQueue;

	// List of New Keyframes
	// TODO add hardcodeo con imágenes de yemas
	std::list<KeyFrame*> mlKeyFramesVotingQueue;

	// Current KeyFrame for one iteration
	KeyFrame* mpCurrentKeyFrame;

	// KeyPoints of Current KeyFrame
	std::vector<cv::KeyPoint> mpCurrentKeyFrameKeyPoints;
	// MapPoints of Current KeyFrame
	std::set<MapPoint*> mpCurrentKeyFrameMapPoints;

	// Descriptors of the keypoints of the current KeyFrame
	cv::Mat mCurrentKeyFrameDescriptors;


	// For reset
	void ResetIfRequested();
	bool mbResetRequested;
	std::mutex mMutexReset;

	// For finish
	bool CheckFinish();
	void SetFinish();
	bool mbFinishRequested;
	bool mbFinished;
	std::mutex mMutexFinish;

	// Imports Another Threads
	LoopClosing* mpLoopCloser;
	LocalMapping* mpLocalMapper;
	Tracking* mpTracker;

	// Flags and mutex for Stop this thread
	std::mutex mMutexStop;
	bool mbStopped;
	bool mbStopRequested;
	bool mbNotStop;

	// Flag for accepting or not new KeyFrames
    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

};

}

#endif /* INCLUDE_VOTING_H_ */
