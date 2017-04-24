/*!
***********************************************************************
*  \file
*     main.c
*  \brief
*     H.264/AVC decoder 18.6 to HM 14.0 
cascade transcoder
*  \author
*     Linwei ZHU
*     <lwzhu2-c@my.cityu.edu.hk>
If you use the code, please cite the following paper
L. Zhu, Y. Zhang, N. Li, G. Jiang, S. Kwong, ¡°Machine Learning Based Fast H.264/AVC to HEVC Transcoding Exploiting Block Partition Similarity¡±, J .Vis. Commun. Image Represent., vol. 38, pp. 824-837, 2016.
***********************************************************************
*/

#include <time.h>
#include "HMsource\TAppEncTop.h"
#include "HMsource\program_options_lite.h"
#include "HMsource\svm.h"

extern "C"
{
#include "JMsource\win32.h"
#include "JMsource\h264decoder.h"
#include "JMsource\configfile.h"
#include "JMsource\mbuffer.h"
#include "JMsource\erc_do.h"
#include "JMsource\mbuffer_common.h"
#include "JMsource\global.h"
};

namespace po = df::program_options_lite;
// #include "contributors.h"
// 
// #include <sys/stat.h>

#define DECOUTPUT_TEST      0

#define PRINT_OUTPUT_POC    0
#define BITSTREAM_FILENAME  "test.264"
#define DECRECON_FILENAME   "test_dec.yuv"
#define ENCRECON_FILENAME   "test_rec.yuv"
#define FCFR_DEBUG_FILENAME "fcfr_dec_rpu_stats.txt"
#define DECOUTPUT_VIEW0_FILENAME  "H264_Decoder_Output_View0.yuv"
#define DECOUTPUT_VIEW1_FILENAME  "H264_Decoder_Output_View1.yuv"


static void Configure(InputParameters *p_Inp, int ac, char *av[])
{
	//char *config_filename=NULL;
	//char errortext[ET_SIZE];
	memset(p_Inp, 0, sizeof(InputParameters));
	strcpy(p_Inp->infile, BITSTREAM_FILENAME); //! set default bitstream name
	strcpy(p_Inp->outfile, DECRECON_FILENAME); //! set default output file name
	strcpy(p_Inp->reffile, ENCRECON_FILENAME); //! set default reference file name

#ifdef _LEAKYBUCKET_
	strcpy(p_Inp->LeakyBucketParamFile,"leakybucketparam.cfg");    // file where Leaky Bucket parameters (computed by encoder) are stored
#endif

	ParseCommand(p_Inp, ac, av);

// 	fprintf(stdout,"----------------------------- JM %s %s -----------------------------\n", VERSION, EXT_VERSION);
// 	//fprintf(stdout," Decoder config file                    : %s \n",config_filename);
// 	if(!p_Inp->bDisplayDecParams)
// 	{
// 		fprintf(stdout,"--------------------------------------------------------------------------\n");
 		fprintf(stdout," Input H.264 bitstream                  : %s \n",p_Inp->infile);
// 		fprintf(stdout," Output decoded YUV                     : %s \n",p_Inp->outfile);
// 		//fprintf(stdout," Output status file                     : %s \n",LOGFILE);
 		fprintf(stdout," Input reference file                   : %s \n",p_Inp->reffile);
// 
// 		fprintf(stdout,"--------------------------------------------------------------------------\n");
// #ifdef _LEAKYBUCKET_
// 		fprintf(stdout," Rate_decoder        : %8ld \n",p_Inp->R_decoder);
// 		fprintf(stdout," B_decoder           : %8ld \n",p_Inp->B_decoder);
// 		fprintf(stdout," F_decoder           : %8ld \n",p_Inp->F_decoder);
// 		fprintf(stdout," LeakyBucketParamFile: %s \n",p_Inp->LeakyBucketParamFile); // Leaky Bucket Param file
// 		calc_buffer(p_Inp);
// 		fprintf(stdout,"--------------------------------------------------------------------------\n");
// #endif
// 	}

}

/*********************************************************
if bOutputAllFrames is 1, then output all valid frames to file onetime; 
else output the first valid frame and move the buffer to the end of list;
*********************************************************/
static int WriteOneFrame(DecodedPicList *pDecPic, int hFileOutput0, int hFileOutput1, int bOutputAllFrames)
{
	int iOutputFrame=0;
	DecodedPicList *pPic = pDecPic;

	if(pPic && (((pPic->iYUVStorageFormat==2) && pPic->bValid==3) || ((pPic->iYUVStorageFormat!=2) && pPic->bValid==1)) )
	{
		int i, iWidth, iHeight, iStride, iWidthUV, iHeightUV, iStrideUV;
		byte *pbBuf;    
		int hFileOutput;
		int res;

		iWidth = pPic->iWidth*((pPic->iBitDepth+7)>>3);
		iHeight = pPic->iHeight;
		iStride = pPic->iYBufStride;
		if(pPic->iYUVFormat != YUV444)
			iWidthUV = pPic->iWidth>>1;
		else
			iWidthUV = pPic->iWidth;
		if(pPic->iYUVFormat == YUV420)
			iHeightUV = pPic->iHeight>>1;
		else
			iHeightUV = pPic->iHeight;
		iWidthUV *= ((pPic->iBitDepth+7)>>3);
		iStrideUV = pPic->iUVBufStride;

		do
		{
			if(pPic->iYUVStorageFormat==2)
				hFileOutput = (pPic->iViewId&0xffff)? hFileOutput1 : hFileOutput0;
			else
				hFileOutput = hFileOutput0;
			if(hFileOutput >=0)
			{
				//Y;
				pbBuf = pPic->pY;
				for(i=0; i<iHeight; i++)
				{
					res = write(hFileOutput, pbBuf+i*iStride, iWidth);
					if (-1==res)
					{
						error ("error writing to output file.", 600);
					}
				}

				if(pPic->iYUVFormat != YUV400)
				{
					//U;
					pbBuf = pPic->pU;
					for(i=0; i<iHeightUV; i++)
					{
						res = write(hFileOutput, pbBuf+i*iStrideUV, iWidthUV);
						if (-1==res)
						{
							error ("error writing to output file.", 600);
						}
					}
					//V;
					pbBuf = pPic->pV;
					for(i=0; i<iHeightUV; i++)
					{
						res = write(hFileOutput, pbBuf+i*iStrideUV, iWidthUV);
						if (-1==res)
						{
							error ("error writing to output file.", 600);
						}
					}
				}

				iOutputFrame++;
			}

			if (pPic->iYUVStorageFormat == 2)
			{
				hFileOutput = ((pPic->iViewId>>16)&0xffff)? hFileOutput1 : hFileOutput0;
				if(hFileOutput>=0)
				{
					int iPicSize =iHeight*iStride;
					//Y;
					pbBuf = pPic->pY+iPicSize;
					for(i=0; i<iHeight; i++)
					{
						res = write(hFileOutput, pbBuf+i*iStride, iWidth);
						if (-1==res)
						{
							error ("error writing to output file.", 600);
						}
					}

					if(pPic->iYUVFormat != YUV400)
					{
						iPicSize = iHeightUV*iStrideUV;
						//U;
						pbBuf = pPic->pU+iPicSize;
						for(i=0; i<iHeightUV; i++)
						{
							res = write(hFileOutput, pbBuf+i*iStrideUV, iWidthUV);
							if (-1==res)
							{
								error ("error writing to output file.", 600);
							}
						}
						//V;
						pbBuf = pPic->pV+iPicSize;
						for(i=0; i<iHeightUV; i++)
						{
							res = write(hFileOutput, pbBuf+i*iStrideUV, iWidthUV);
							if (-1==res)
							{
								error ("error writing to output file.", 600);
							}
						}
					}

					iOutputFrame++;
				}
			}

#if PRINT_OUTPUT_POC
			fprintf(stdout, "\nOutput frame: %d/%d\n", pPic->iPOC, pPic->iViewId);
#endif
			pPic->bValid = 0;
			pPic = pPic->pNext;
		}while(pPic != NULL && pPic->bValid && bOutputAllFrames);
	} 
#if PRINT_OUTPUT_POC
	else
		fprintf(stdout, "\nNone frame output\n");
#endif

	return iOutputFrame;
}


#include <opencv2/opencv.hpp>
#include <opencv2/highgui//highgui.hpp>
#include <opencv2/ml/ml.hpp>
using namespace cv;   

int trainingGOP=1;
#define  TRAINING 1

int main(int argc, char **argv)
{
	cout<<"JM18.6 TO HM14.0  version-2014-06-06 proposed"<<endl;
	//========JM18.4 decode===========
	int iRet;
	DecodedPicList *pDecPicList;
	int hFileDecOutput0=-1, hFileDecOutput1=-1;
	int iFramesOutput=0, iFramesDecoded=0;
	InputParameters InputParams;

	TAppEncTop  cTAppEncTop;//HEVC
	double HEVCtime=0;

#if DECOUTPUT_TEST
	hFileDecOutput0 = open(DECOUTPUT_VIEW0_FILENAME, OPENFLAGS_WRITE, OPEN_PERMISSIONS);
	fprintf(stdout, "Decoder output view0: %s\n", DECOUTPUT_VIEW0_FILENAME);
	hFileDecOutput1 = open(DECOUTPUT_VIEW1_FILENAME, OPENFLAGS_WRITE, OPEN_PERMISSIONS);
	fprintf(stdout, "Decoder output view1: %s\n", DECOUTPUT_VIEW1_FILENAME);
#endif

	// create application encoder class
	cTAppEncTop.create();//HEVC
	Int   iNumEncoded = 0;
	list<AccessUnit> outputAccessUnits; ///< list of access units to write out.  is populated by the encoding process
	TComPicYuv*       pcPicYuvOrg = new TComPicYuv;
	TComPicYuv*       pcPicYuvRec = NULL;

	int feature=13;//number of feature
	int frameSize0,frameSize1,frameSize2;
	//=============================================================================================================
	double start=clock();

	init_time();

	//get input parameters;
	Configure(&InputParams, argc, argv);
	// parse configuration
	try
	{
		if(!cTAppEncTop.parseCfg( argc, argv ))
		{
			cTAppEncTop.destroy();
			return 1;
		}
	}
	catch (po::ParseFailure& e)
	{
		cerr << "Error parsing option \""<< e.arg <<"\" with argument \""<< e.val <<"\"." << endl;
		return 1;
	}
	//---------------------------------------------------------------------------------------------------------
	int WIDTH=cTAppEncTop.GetWidth();
	int HEIGHT=cTAppEncTop.GetHeight();

	unsigned char *tempY=new unsigned char[WIDTH*HEIGHT];
	unsigned char *tempU=new unsigned char[WIDTH*HEIGHT/4];
	unsigned char *tempV=new unsigned char[WIDTH*HEIGHT/4];

	unsigned char *previousY=new unsigned char[WIDTH*HEIGHT];
	//--------------------------boundary----------------------------------------------
	if (WIDTH%64==0&&HEIGHT%64==0) 
		frameSize0 = WIDTH/64*HEIGHT/64;
	else if (WIDTH%64!=0&&HEIGHT%64==0)
		frameSize0 = (WIDTH/64+1)*HEIGHT/64;
	else if (WIDTH%64==0&&HEIGHT%64!=0)
		frameSize0 = WIDTH/64*(HEIGHT/64+1);
	else if (WIDTH%64!=0&&HEIGHT%64!=0)
		frameSize0 = (WIDTH/64+1)*(HEIGHT/64+1);
	//-------------------------------------------------------------------------------------------------
	frameSize1=4*frameSize0;frameSize2=16*frameSize0;
	//=========================================================================================================
	int W0,W1,W2,H0,H1,H2;

	if (WIDTH%64==0)  W0 = WIDTH/64;
	else              W0 = WIDTH/64+1;

	W1=2*W0;W2=4*W0;

	if (HEIGHT%64==0) H0 = HEIGHT/64;
	else              H0 = HEIGHT/64+1;

	H1=2*H0;H2=4*H0;
	int trainingframe = trainingGOP*cTAppEncTop.GetGop();
	//---------------------------------------------------------------------------------------------------------
	CvMat *Feature0=cvCreateMat(frameSize0*trainingframe,feature,CV_64FC1);cvZero(Feature0);
	CvMat *Feature1=cvCreateMat(frameSize1*trainingframe,feature,CV_64FC1);cvZero(Feature1);
	CvMat *Feature2=cvCreateMat(frameSize2*trainingframe,feature,CV_64FC1);cvZero(Feature2);

	CvMat *TrueDepth0=cvCreateMat(frameSize0*trainingframe,1,CV_64FC1);cvZero(TrueDepth0);//HEVC-coder
	CvMat *TrueDepth1=cvCreateMat(frameSize1*trainingframe,1,CV_64FC1);cvZero(TrueDepth1);//HEVC-coder
	CvMat *TrueDepth2=cvCreateMat(frameSize2*trainingframe,1,CV_64FC1);cvZero(TrueDepth2);//HEVC-coder

	CvMat *Feature00=cvCreateMat(frameSize0*cTAppEncTop.GetGop(),feature,CV_64FC1);cvZero(Feature00);
	CvMat *Feature11=cvCreateMat(frameSize1*cTAppEncTop.GetGop(),feature,CV_64FC1);cvZero(Feature11);
	CvMat *Feature22=cvCreateMat(frameSize2*cTAppEncTop.GetGop(),feature,CV_64FC1);cvZero(Feature22);

	maxmin *M0,*M1,*M2;
	M0=new maxmin [feature];
	M1=new maxmin [feature];
	M2=new maxmin [feature];
//===================================================================================================================
	fstream bitstreamFile(cTAppEncTop.GetBitstreamFile(), fstream::binary | fstream::out);
	if (!bitstreamFile)
	{
		fprintf(stderr, "\nfailed to open bitstream file `%s' for writing\n", cTAppEncTop.GetBitstreamFile());
		exit(EXIT_FAILURE);
	}
	//open decoder;
	iRet = OpenDecoder(&InputParams);
	cTAppEncTop.Initencode(cTAppEncTop.Getfield());//zhulinwei
	if(iRet != DEC_OPEN_NOERR)
	{
		fprintf(stderr, "Open encoder failed: 0x%x!\n", iRet);
		return -1; //failed;
	}

	svm_model *model0,*model1,*model2;

	bool Intraflag = true,flagtraining = true;int indexGop=0;

	//decoding;
	do
	{
		if (flagtraining)
		{
			if (Intraflag)
			{
				iRet = DecodeOneFrame(&pDecPicList,Feature0,Feature1,Feature2,-1);
			}
			else
			{
				iRet = DecodeOneFrame(&pDecPicList,Feature0,Feature1,Feature2,iFramesDecoded-1);
			}

			if(iRet==DEC_EOS || iRet==DEC_SUCCEED)
			{
				//process the decoded picture, output or display;
				iFramesOutput += WriteOneFrame(pDecPicList, hFileDecOutput0, hFileDecOutput1, 0);
				iFramesDecoded++;
			}
			else
			{
				//error handling;
				fprintf(stderr, "Error in decoding process: 0x%x\n", iRet);
			}
			DecodedPictureBuffer *temp_buffer=p_Dec->p_Vid->p_Dpb_layer[0];
			VideoParameters *p_Vid = temp_buffer->p_Vid;
			// diagnostics
			// printf("Flush remaining frames from the dpb. p_Dpb->size=%d, p_Dpb->used_size=%d\n",p_Dpb->size,p_Dpb->used_size);
			if(!temp_buffer->init_done)
				return 0 ;
			//  if(p_Vid->conceal_mode == 0)
			if (p_Vid->conceal_mode != 0)
				conceal_non_ref_pics(temp_buffer, 0);

			// mark all frames unused
			for (int i=0; i<temp_buffer->used_size; i++)
			{
#if MVC_EXTENSION_ENABLE
				assert( temp_buffer->fs[i]->view_id == temp_buffer->layer_id);
#endif
				//unmark_for_reference (temp_buffer->fs[i]);
			}
			while (remove_unused_frame_from_dpb(temp_buffer));
			//============================================================================================================
			// get frames in POC order
			while (temp_buffer->used_size && get_one_frame_from_dpb(temp_buffer,tempY,tempU,tempV))
			{
				if (!Intraflag)
				{
					//==========================================================================================
					int x0=0,x1=0,x2=0;
							//------------------------------64X64-----------------------------------------------------------
							for (int i=0;i<HEIGHT;i+=64)
								for(int j=0;j<WIDTH;j+=64)
								{
									double sum=0;
										for (int m=i;m<64+i;m++)
											for(int n=j;n<64+j;n++)
											{
												int p=m;
												if(p>=HEIGHT) p=2*HEIGHT-p-1;
												int q=n;
												if(q>=WIDTH)  q=2*WIDTH-q-1;
												//--------------------------------------------------------
												sum+=abs(previousY[p*WIDTH+q]-tempY[p*WIDTH+q]);
											}
											cvmSet(Feature0,x0+(iFramesDecoded-2)*frameSize0,0,sum/(64*64));
											x0++;
								}
								//-------------------------------32X32------------------------------------------------------------------
								for (int i=0;i<HEIGHT;i+=32)
									for(int j=0;j<WIDTH;j+=32)
									{
										double sum=0;
											for (int m=i;m<32+i;m++)
												for(int n=j;n<32+j;n++)
												{
													int p=m;
													if(p>=HEIGHT) p=2*HEIGHT-p-1;
													int q=n;
													if(q>=WIDTH)  q=2*WIDTH-q-1;
													//--------------------------------------------------------
													sum+=abs(previousY[p*WIDTH+q]-tempY[p*WIDTH+q]);
												}
												cvmSet(Feature1,x1+(iFramesDecoded-2)*frameSize1,0,sum/(32*32));
												x1++;
									}
									//-------------------------------16X16--------------------------------------------------------------------
									for (int i=0;i<HEIGHT;i+=16)
										for(int j=0;j<WIDTH;j+=16)
										{
											double sum=0;
												for (int m=i;m<16+i;m++)
													for(int n=j;n<16+j;n++)
													{
														int p=m;
														if(p>=HEIGHT) p=2*HEIGHT-p-1;
														int q=n;
														if(q>=WIDTH)  q=2*WIDTH-q-1;
														//--------------------------------------------------------
														sum+=abs(previousY[p*WIDTH+q]-tempY[p*WIDTH+q]);
													}
													cvmSet(Feature2,x2+(iFramesDecoded-2)*frameSize2,0,sum/(16*16));
													x2++;
										}
				}
				memcpy(previousY,tempY,sizeof(unsigned char)*WIDTH*HEIGHT);

				cTAppEncTop.YuvOrgCrate(pcPicYuvOrg,cTAppEncTop.Getfield());
				cTAppEncTop.ReadDEYuv(pcPicYuvOrg,tempY,tempU,tempV);
				// 	starting time
				double HEVCstart = clock();
				if (Intraflag)
				{
					cTAppEncTop.encode1(pcPicYuvRec,pcPicYuvOrg,outputAccessUnits,iNumEncoded,bitstreamFile,
						               Feature0,Feature1,Feature2,TrueDepth0,TrueDepth1,TrueDepth2,Intraflag,indexGop);
					Intraflag = false;
				}
				else
				{
					cTAppEncTop.encode1(pcPicYuvRec,pcPicYuvOrg,outputAccessUnits,iNumEncoded,bitstreamFile,
						               Feature0,Feature1,Feature2,TrueDepth0,TrueDepth1,TrueDepth2,Intraflag,indexGop);
					if ((iFramesDecoded-1)%cTAppEncTop.GetGop()==0)
					{
						indexGop++;
					}
				}
				double HEVCend = clock();
				HEVCtime+=HEVCend-HEVCstart;
			};
			temp_buffer->last_output_poc = INT_MIN;

			if ((iFramesDecoded-1)/cTAppEncTop.GetGop()==trainingGOP)
			{
				flagtraining = false;
			}
	//--------------------------------------training----------------------------------------------------//
			if (!flagtraining)
			{
				cout<<"Training....Please waiting..."<<endl;
				//---------------------------------------------------------------
				struct svm_parameter param;		// set by parse_command_line
				struct svm_problem prob0;		// set by read_problem
				struct svm_problem prob1;		// set by read_problem
				struct svm_problem prob2;		// set by read_problem
				struct svm_node *data0,*data1,*data2;

				param.svm_type=C_SVC;
				param.C=1;
				param.degree=5;
				param.kernel_type=RBF;
				param.eps=1e-6;
				param.gamma=0.5;

				param.coef0 = 0;
				param.nu = 0.5;
				param.cache_size = 100;
				param.p = 0.1;
				param.shrinking = 1;
				param.probability = 0;
				param.nr_weight = 0;
				param.weight_label = NULL;
				param.weight = NULL;
				//==============================normailzation============================================================
				for (int j=0;j<feature;j++)
				{
					M0[j].maxvalue=cvmGet(Feature0,0,j);
					M0[j].minvalue=cvmGet(Feature0,0,j);

					M1[j].maxvalue=cvmGet(Feature1,0,j);
					M1[j].minvalue=cvmGet(Feature1,0,j);

					M2[j].maxvalue=cvmGet(Feature2,0,j);
					M2[j].minvalue=cvmGet(Feature2,0,j);
				}
				//=========================================================================================
				for (int i=0;i<frameSize0*trainingframe;i++)
				{
					for (int j=0;j<feature;j++)
					{
						if (cvmGet(Feature0,i,j)>=M0[j].maxvalue)     M0[j].maxvalue=cvmGet(Feature0,i,j);
						else if(cvmGet(Feature0,i,j)<=M0[j].minvalue) M0[j].minvalue=cvmGet(Feature0,i,j);
					}
				}
				//------------------------------------------------------------------------------------------------
				for (int frame=0;frame<trainingframe;frame++)
				{
					for (int i=0;i<frameSize0;i++)
					{
						int tempx=2*(i/W0),tempy=2*(i%W0);
						for (int l=0;l<4;l++)
						{
							int temp1=W1*(tempx+l/2)+(tempy+l%2);
							if(0!=cvmGet(TrueDepth1,4*i+l+frame*frameSize1,0)) 
							{
								for (int j=0;j<feature;j++)
								{
									if (cvmGet(Feature1,temp1+frame*frameSize1,j)>=M1[j].maxvalue)     M1[j].maxvalue=cvmGet(Feature1,temp1+frame*frameSize1,j);
									else if(cvmGet(Feature1,temp1+frame*frameSize1,j)<=M1[j].minvalue) M1[j].minvalue=cvmGet(Feature1,temp1+frame*frameSize1,j);
								}
							}
						}
					}
				}
				//---------------------------------------------------------------------------------------------------------
				for (int frame=0;frame<trainingframe;frame++)
				{
					for (int i=0;i<frameSize0;i++)
					{
						int tempx=2*(i/W0),tempy=2*(i%W0);
						for (int l=0;l<4;l++)
						{
							int tempxx=2*tempx+2*(l/2),tempyy=2*tempy+2*(l%2);
							for (int m=0;m<4;m++)
							{
								int temp2=W2*(tempxx+m/2)+(tempyy+m%2);
								if(0!=cvmGet(TrueDepth2,4*(4*i+l)+m+frame*frameSize2,0)) 
								{
									for (int j=0;j<feature;j++)
									{
										if (cvmGet(Feature2,temp2+frame*frameSize2,j)>=M2[j].maxvalue)     M2[j].maxvalue=cvmGet(Feature2,temp2+frame*frameSize2,j);
										else if(cvmGet(Feature2,temp2+frame*frameSize2,j)<=M2[j].minvalue) M2[j].minvalue=cvmGet(Feature2,temp2+frame*frameSize2,j);
									}
								}
							}

						}
					}
				}
				//====================================================================================================================
				int count1=0;int count2=0;
				int W00=WIDTH/64,H00=HEIGHT/64;
				int W11=WIDTH/32,H11=HEIGHT/32;
				int W22=WIDTH/16,H22=HEIGHT/16;
				for (int Class=0;Class<3;Class++)
				{
					//=================================================
					int n=0;
					//=================================================
					switch(Class)
					{
					case 0:
						prob0.l=frameSize0*trainingframe;
						prob0.y=new double [frameSize0*trainingframe];
						prob0.x=new svm_node *[frameSize0*trainingframe];
						data0=new svm_node[frameSize0*trainingframe*(feature+1)];
						//-----------------------------------------------------------------------------------------------------
						for (int frame=0,k=0;frame<trainingframe;frame++)
						{
							for (int i=0;i<frameSize0;i++)
							{			
								prob0.x[n]=&data0[k];
								if(0!=cvmGet(TrueDepth0,i+frame*frameSize0,0)) 
								{
									prob0.y[n]=cvmGet(TrueDepth0,i+frame*frameSize0,0);
									if (i/W0>=H00||i%W0>=W00)
									{
										if(prob0.y[n]==-1) prob0.y[n]=1;
									}
								}

								if (0!=cvmGet(TrueDepth0,i+frame*frameSize0,0))
								{
									for (int j=0;j<feature;j++)
									{
										data0[k].index=j+1;
										data0[k].value=-1+2*(cvmGet(Feature0,i+frame*frameSize0,j)-M0[j].minvalue)/(M0[j].maxvalue-M0[j].minvalue+1e-8);
										if (i/W0>=H00||i%W0>=W00)
										{
											data0[k].value=1;
										}
										k++;
									}
									data0[k++].index=-1;
									n++;
								}
							}
						}
						break;
						//-----------------------------------------------------------------------
					case 1:
						for (int i=0;i<frameSize1*trainingframe;i++)
						{
							if(0!=cvmGet(TrueDepth1,i,0)) count1++;
						}
						prob1.l=count1;
						prob1.y=new double [count1];
						prob1.x=new svm_node *[count1];
						data1=new svm_node[count1*(feature+1)];
						//------------------------------------------------------------------------
						for (int frame=0,k=0;frame<trainingframe;frame++)
						{
							for (int i=0;i<frameSize0;i++)
							{
								int tempx=2*(i/W0),tempy=2*(i%W0);
								for (int l=0;l<4;l++)
								{
									int temp1=W1*(tempx+l/2)+(tempy+l%2);
									if(0!=cvmGet(TrueDepth1,4*i+l+frame*frameSize1,0)) 
									{
										prob1.y[n]=cvmGet(TrueDepth1,4*i+l+frame*frameSize1,0);
										if (temp1/W1>=H11||temp1%W1>=W11)
										{
											if(prob1.y[n]==-1) prob1.y[n]=1;
										}
									}

									if(0!=cvmGet(TrueDepth1,4*i+l+frame*frameSize1,0))
									{
										prob1.x[n]=&data1[k];
										for (int j=0;j<feature;j++)
										{
											data1[k].index=j+1;
											data1[k].value=-1+2*(cvmGet(Feature1,temp1+frame*frameSize1,j)-M1[j].minvalue)/(M1[j].maxvalue-M1[j].minvalue+1e-8);
											if (temp1/W1>=H11||temp1%W1>=W11)
											{
												data1[k].value=1;
											}
											k++;
										}
										data1[k++].index=-1;
										n++;
									}

								}
							}
						}
						break;
					case 2:
						for (int i=0;i<frameSize2*trainingframe;i++)
						{
							if(0!=cvmGet(TrueDepth2,i,0)) count2++;
						}
						prob2.l=count2;
						prob2.y=new double [count2];
						prob2.x=new svm_node *[count2];
						data2=new svm_node[count2*(feature+1)];
						//-------------------------------------------------------------
						for (int frame=0,k=0;frame<trainingframe;frame++)
						{
							for (int i=0;i<frameSize0;i++)
							{
								int tempx=2*(i/W0),tempy=2*(i%W0);
								for (int l=0;l<4;l++)
								{
									int tempxx=2*tempx+2*(l/2),tempyy=2*tempy+2*(l%2);
									for (int m=0;m<4;m++)
									{
										int temp2=W2*(tempxx+m/2)+(tempyy+m%2);
										if(0!=cvmGet(TrueDepth2,4*(4*i+l)+m+frame*frameSize2,0)) 
										{
											prob2.y[n]=cvmGet(TrueDepth2,4*(4*i+l)+m+frame*frameSize2,0);
											if (temp2/W2>=H22||temp2%W2>=W22)
											{
												if(prob2.y[n]==-1) prob2.y[n]=1;
											}
										}

										if(0!=cvmGet(TrueDepth2,4*(4*i+l)+m+frame*frameSize2,0))
										{
											prob2.x[n]=&data2[k];
											for (int j=0;j<feature;j++)
											{
												data2[k].index=j+1;
												data2[k].value=-1+2*(cvmGet(Feature2,temp2+frame*frameSize2,j)-M2[j].minvalue)/(M2[j].maxvalue-M2[j].minvalue+1e-8);
												if (temp2/W2>=H22||temp2%W2>=W22)
												{
													data2[k].value=1;
												}
												k++;
											}

											data2[k++].index=-1;

											n++;
										}

									}

								}
							}
						}
						break;
					}
					//=====================================================================================================
#if TRAINING
					const char *error_msg=NULL;
					switch(Class)
					{
					case 0:
						error_msg=svm_check_parameter(&prob0,&param);
						if(error_msg)
						{
							fprintf(stderr,"ERROR: %s\n",error_msg);
							exit(1);
						}
						model0=svm_train(&prob0,&param);
						//svm_save_model("model0.txt",model0);
						delete [] prob0.x;
						delete [] prob0.y;
						break;
					case 1:
						error_msg=svm_check_parameter(&prob1,&param);
						if(error_msg)
						{
							fprintf(stderr,"ERROR: %s\n",error_msg);
							exit(1);
						}
						model1=svm_train(&prob1,&param);
						//svm_save_model("model1.txt",model1);
						delete [] prob1.x;
						delete [] prob1.y;
						break;
					case 2:
						error_msg=svm_check_parameter(&prob2,&param);
						if(error_msg)
						{
							fprintf(stderr,"ERROR: %s\n",error_msg);
							exit(1);
						}
						model2=svm_train(&prob2,&param);
						//svm_save_model("model2.txt",model2);
						delete [] prob2.x;
						delete [] prob2.y;
						break;
					}
#endif

					//=====================================================================================================
				}

				cout<<"Training End....."<<endl;
				fprintf(stdout,"--------------------------------------------------------------------------\n");
			}

	//--------------------------------------training----------------------------------------------------//
		}
		else
		{
			int index0 = (iFramesDecoded-1)%cTAppEncTop.GetGop();
			iRet = DecodeOneFrame(&pDecPicList,Feature00,Feature11,Feature22,index0);
			if(iRet==DEC_EOS || iRet==DEC_SUCCEED)
			{
				//process the decoded picture, output or display;
				iFramesOutput += WriteOneFrame(pDecPicList, hFileDecOutput0, hFileDecOutput1, 0);
				iFramesDecoded++;
			}
			else
			{
				//error handling;
				fprintf(stderr, "Error in decoding process: 0x%x\n", iRet);
			}
			DecodedPictureBuffer *temp_buffer=p_Dec->p_Vid->p_Dpb_layer[0];
			VideoParameters *p_Vid = temp_buffer->p_Vid;
			// diagnostics
			// printf("Flush remaining frames from the dpb. p_Dpb->size=%d, p_Dpb->used_size=%d\n",p_Dpb->size,p_Dpb->used_size);
			if(!temp_buffer->init_done)
				return 0 ;
			//  if(p_Vid->conceal_mode == 0)
			if (p_Vid->conceal_mode != 0)
				conceal_non_ref_pics(temp_buffer, 0);

			// mark all frames unused
			for (int i=0; i<temp_buffer->used_size; i++)
			{
#if MVC_EXTENSION_ENABLE
				assert( temp_buffer->fs[i]->view_id == temp_buffer->layer_id);
#endif
				//unmark_for_reference (temp_buffer->fs[i]);
			}
			while (remove_unused_frame_from_dpb(temp_buffer));
			//============================================================================================================
			// get frames in POC order
			while (temp_buffer->used_size && get_one_frame_from_dpb(temp_buffer,tempY,tempU,tempV))
			{
				int x0=0,x1=0,x2=0;
			   //------------------------------64X64-----------------------------------------------------------
				for (int i=0;i<HEIGHT;i+=64)
					for(int j=0;j<WIDTH;j+=64)
					{
						double sum=0;
						for (int m=i;m<64+i;m++)
						     for(int n=j;n<64+j;n++)
							{
								int p=m;
								if(p>=HEIGHT) p=2*HEIGHT-p-1;
								int q=n;
								if(q>=WIDTH)  q=2*WIDTH-q-1;
						        //--------------------------------------------------------
								sum+=abs(previousY[p*WIDTH+q]-tempY[p*WIDTH+q]);
							}
						cvmSet(Feature00,x0+index0*frameSize0,0,sum/(64*64));
						x0++;
					}
				//-------------------------------32X32--------------------------------------------------------------------
				for (int i=0;i<HEIGHT;i+=32)
					for(int j=0;j<WIDTH;j+=32)
					{
						double sum=0;
						for (int m=i;m<32+i;m++)
							for(int n=j;n<32+j;n++)
						    {
								int p=m;
								if(p>=HEIGHT) p=2*HEIGHT-p-1;
								int q=n;
								if(q>=WIDTH)  q=2*WIDTH-q-1;
								//--------------------------------------------------------
								sum+=abs(previousY[p*WIDTH+q]-tempY[p*WIDTH+q]);
							}
						cvmSet(Feature11,x1+index0*frameSize1,0,sum/(32*32));
						x1++;
					}
				//-------------------------------16X16--------------------------------------------------------------------
				for (int i=0;i<HEIGHT;i+=16)
					for(int j=0;j<WIDTH;j+=16)
					{
						double sum=0;
						for (int m=i;m<16+i;m++)
							for(int n=j;n<16+j;n++)
							{
								int p=m;
								if(p>=HEIGHT) p=2*HEIGHT-p-1;
								int q=n;
								if(q>=WIDTH)  q=2*WIDTH-q-1;
								//--------------------------------------------------------
								sum+=abs(previousY[p*WIDTH+q]-tempY[p*WIDTH+q]);
							}
						cvmSet(Feature22,x2+index0*frameSize2,0,sum/(16*16));
						x2++;
					}

				cTAppEncTop.YuvOrgCrate(pcPicYuvOrg,cTAppEncTop.Getfield());
				cTAppEncTop.ReadDEYuv(pcPicYuvOrg,tempY,tempU,tempV);
				// 	starting time
				double HEVCstart = clock();
				cTAppEncTop.encode2(pcPicYuvRec,pcPicYuvOrg,outputAccessUnits,iNumEncoded,bitstreamFile,
					                Feature00,Feature11,Feature22,model0,model1,model2,M0,M1,M2);
				double HEVCend = clock();
				HEVCtime+=HEVCend-HEVCstart;
			};
			temp_buffer->last_output_poc = INT_MIN;
		}
	}while((iRet == DEC_SUCCEED) && ((p_Dec->p_Inp->iDecFrmNum==0) || (iFramesDecoded<p_Dec->p_Inp->iDecFrmNum)));

	delete [] tempY;delete [] tempU;delete [] tempV;//zhulinwei
	//pcPicYuvOrg->destroy();
	delete pcPicYuvOrg;
	pcPicYuvOrg = NULL;
	cTAppEncTop.DeleteBuffer();
	//iRet = FinitDecoder(&pDecPicList);
	iFramesOutput += WriteOneFrame(pDecPicList, hFileDecOutput0, hFileDecOutput1 , 1);
	iRet = CloseDecoder();

	//quit;
	if(hFileDecOutput0>=0)
	{
		close(hFileDecOutput0);
	}
	if(hFileDecOutput1>=0)
	{
		close(hFileDecOutput1);
	}
	//===============================================================================================================
	double end=clock();

	HEVCtime=HEVCtime*0.001;
	cout<<"Total HEVC encoding time: "<<HEVCtime<<" sec ("<<HEVCtime/iFramesDecoded<<" fps)"<<endl;
	fprintf(stdout,"--------------------------------------------------------------------------\n");
	end=(end-start)*0.001;
	cout<<"Total transcoding time: "<<end<<" sec "<<endl;

	printf("%d frames are transcoded.\n", iFramesDecoded);
	return 0;
}


