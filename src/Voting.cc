
/*
 *
 *  New Thread for Voting Scheme
 *  Author: Betetto FRanco
 */
#include "Voting.h"
#include<iostream>

#include <mutex>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

namespace ORB_SLAM2
{
Voting::Voting():
		// Initialize the flags
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
			cout << "KeyFrame detected on Voting" << contador << endl;
			contador += 1;
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

