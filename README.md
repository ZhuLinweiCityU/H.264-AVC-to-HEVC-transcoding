# H.264-AVC-to-HEVC-transcoding
This is the source code of H.264/AVC to HEVC transcoding. If you use the code, please cite the following paper.
L. Zhu, Y. Zhang, N. Li, G. Jiang, S. Kwong, “Machine Learning Based Fast H.264/AVC to HEVC Transcoding Exploiting Block Partition Similarity”, J .Vis. Commun. Image Represent., vol. 38, pp. 824-837, 2016.
In the file of original_JM186_HM140, it is the source code of original transcoder, combining the JM decoder and HM encoder without any optimization.
In the file of transcode_proposed, it is the source code of proposed method in the paper.
In the file of test_document, there is an example for transcoding.
It should be noted that (1) the opencv is supposed to be equipped for the proposed method;
                        (2) the number of decoding should equal to the number of encoding;
                        (3) low delay P main and low delay main configurations are suitable
                        (4) for random access configuration, the code maybe done some changes because of I frame.
If you have any questions, please feel free to contact me. (lwzhu2-c@my.cityu.edu.hk)
