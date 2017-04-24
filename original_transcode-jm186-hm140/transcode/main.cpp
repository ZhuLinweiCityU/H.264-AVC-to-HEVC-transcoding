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
***********************************************************************
*/

#include <time.h>
#include "HMsource\TAppEncTop.h"
#include "HMsource\program_options_lite.h"

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

/*!
***********************************************************************
* \brief
*    main function for JM decoder
***********************************************************************
*/
int main(int argc, char **argv)
{
	cout<<"JM18.6 TO HM14.0  version20140516 cascade"<<endl;
	//========JM18.4decoding===========
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
	//=============================================================================================================
	double start=clock();//

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
	//===========================================================
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

	//decoding;
	do
	{
		iRet = DecodeOneFrame(&pDecPicList);
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
			cTAppEncTop.YuvOrgCrate(pcPicYuvOrg,cTAppEncTop.Getfield());
			cTAppEncTop.ReadDEYuv(pcPicYuvOrg,tempY,tempU,tempV);
			// 	starting time
			double HEVCstart = clock();
			cTAppEncTop.encode(pcPicYuvRec,pcPicYuvOrg,outputAccessUnits,iNumEncoded,bitstreamFile);
			double HEVCend = clock();
			HEVCtime+=HEVCend-HEVCstart;
		};
		temp_buffer->last_output_poc = INT_MIN;
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


