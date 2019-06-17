#pragma once


//  output is a binary image
//  1: not a min region
//  0: part of a min region
//  2: not sure if min or not
//  3: uninitialized


int inline neighborCleanup(float* in, uchar* out, int i, int x, int y, int x_lim, int y_lim)
{
    int index;
    for (int xx = x - 1; xx < x + 2; ++xx) {
        for (int yy = y - 1; yy < y + 2; ++yy) {
            if (((xx == x) && (yy==y)) || xx < 0 || yy < 0 || xx >= x_lim || yy >= y_lim)
                continue;
            index = xx*y_lim + yy;
            if ((in[i] == in[index]) && (out[index] == 0))
                return 1;
        }
    }

    return 0;
}

void inline neighborCheck(float* in, uchar* out, int i, int x, int y, int x_lim)
{   
    int indexes[8], cur_index;
    indexes[0] = x*x_lim + y;
    indexes[1] = x*x_lim + y+1;
    indexes[2] = x*x_lim + y+2;
    indexes[3] = (x+1)*x_lim + y+2;
    indexes[4] = (x + 2)*x_lim + y+2;
    indexes[5] = (x + 2)*x_lim + y + 1;
    indexes[6] = (x + 2)*x_lim + y;
    indexes[7] = (x + 1)*x_lim + y;
    cur_index = (x + 1)*x_lim + y+1;

    for (int t = 0; t < 8; t++) {
        if (in[indexes[t]] < in[cur_index]) {
            out[i] = 0;
            break;
        }
    }

    if (out[i] == 3)
        out[i] = 1;
}


// inpridred by matlab imregionalmin
Eigen::MatrixXf imregionalmin(Eigen::MatrixXf& matEigen) // cv::Mat out_mat
{
    cv::Mat out_mat;
    matEigen=-matEigen;
    cv::Mat mat;
    cv::eigen2cv(matEigen,mat);
    // pad the border of mat with 1 and copy to img_pad
    cv::Mat img_pad;
    cv::copyMakeBorder(mat, img_pad, 1, 1, 1, 1, IPL_BORDER_CONSTANT, 1);

    //  initialize binary output to 2, unknown if min
    out_mat = cv::Mat::ones(mat.rows, mat.cols, CV_8U)+2;

    //  initialize pointers to matrices
    float* in = (float *)(img_pad.data);
    uchar* out = (uchar *)(out_mat.data);

    //  size of matrix
    int in_size = img_pad.cols*img_pad.rows;
    int out_size = mat.cols*mat.rows;

    int x, y;
    for (int i = 0; i < out_size; i++) {
        //  find x, y indexes
        y = i % mat.cols;
        x = i / mat.cols;

        neighborCheck(in, out, i, x, y, img_pad.cols);  //  all regions are either min or max
    }

    cv::Mat label;
    cv::connectedComponents(out_mat, label);

    int* lab = (int *)(label.data);

    in = (float *)(mat.data);
    in_size = mat.cols*mat.rows;

    std::vector<int> bad_labels;

    for (int i = 0; i < out_size; i++) {
        //  find x, y indexes
        y = i % mat.cols;
        x = i / mat.cols;

        if (lab[i] != 0) {
            if (neighborCleanup(in, out, i, x, y, mat.rows, mat.cols) == 1) {
                bad_labels.push_back(lab[i]);
            }
        }
    }

    std::sort(bad_labels.begin(), bad_labels.end());
    bad_labels.erase(std::unique(bad_labels.begin(), bad_labels.end()), bad_labels.end());

    for (int i = 0; i < out_size; ++i) {
        if (lab[i] != 0) {
            if (std::find(bad_labels.begin(), bad_labels.end(), lab[i]) != bad_labels.end()) {
                out[i] = 0;
            }
        }
    }

//    std::cout << "M = "<< std::endl << " "  << out_mat << std::endl << std::endl;

    Eigen::MatrixXf out_mat_eigen;
    cv::cv2eigen(out_mat,out_mat_eigen);
    return out_mat_eigen;

}

