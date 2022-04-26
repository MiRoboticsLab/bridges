#include "cyberdog_ota/client.hpp"
#include "cyberdog_ota/server.hpp"
#include "cyberdog_ota/utils.hpp"

#include <iostream>

namespace cyberdog
{

int Client::abort_download = 0;

struct ProgressData
{
    Server* server;
    long long total_file_size;
    long long cur_file_size;
    CURL* curl;
    bool resume;
};

Client::Client()
{

}


Client::~Client()
{

}


bool Client::Initialize()
{
  return true;
}

static size_t WriteFileCallback(char* ptr, size_t size, size_t nmemb, FILE* userdata)
{
    return fwrite(ptr, size, nmemb, userdata);
}

int Client::ProgressCallback(void* clientp, double dltotal, double dlnow, double ultotal, double ulnow)
{
    ProgressData* progress_data = (ProgressData*)clientp;
    double now = dlnow;
    double total = dltotal;
    if (progress_data->resume)
    {
        now += (double)progress_data->cur_file_size;
        total = (double)progress_data->total_file_size;
    }

    if (total != 0)
    {
        int temp = (int)((now * 1.0 / total) * 100);
        if (temp >= 100)
        {
            temp = 99;
        }

        // TODO
        // progress_data->my_server->m_download_progress = temp;

        double speed = 0.0;
        curl_easy_getinfo(progress_data->curl, CURLINFO_SPEED_DOWNLOAD, &speed);
        if (speed != 0)
        {
            int temp_time = (int)((total - now) / speed);
            if (temp_time == 0)
            {
                temp_time = 1;
            }

            // TODO
            // progress_data->my_server->m_download_time = temp_time;
        }

        ///printf("dltotal = %f, dlnow = %f, ultotal = %f, ulnow = %f, speed = %f, progress = %d\n",
            ///dltotal, dlnow, ultotal, ulnow, speed, progress_data->my_server->m_download_progress);
    }

    return abort_download;
}

int Client::Download(void* user_data, const std::string& url, const std::string& path, bool resume, const char* ca_path)
{
    ProgressData progress_data;
    long response_code = 206;
    FILE* fp = NULL;
    if (resume)
    {
        long long total_file_size = 0;
        int ret = GetFileSize(url, total_file_size, ca_path);
        long long cur_file_size = FileSize(path.c_str());
        if ((ret < 0) || (cur_file_size < 0) || (cur_file_size > total_file_size))
        {
            progress_data.resume = false;
            fp = fopen(path.c_str(), "wb");
        }
        else
        {
            if (cur_file_size == total_file_size)
            {
                return 0;
            }
            progress_data.resume = true;
            progress_data.cur_file_size = cur_file_size;
            progress_data.total_file_size = total_file_size;
            fp = fopen(path.c_str(), "ab+");
        }
    }
    else
    {
        response_code = 200;
        progress_data.resume = false;
        fp = fopen(path.c_str(), "wb");
    }

    if (NULL == fp)
    {
        // LOGE("download error path = %s", path.c_str());
        std::cout << "download error path = " << path.c_str() << std::endl;
        return -1;
    }

    CURL* curl = curl_easy_init();
    if (NULL == curl)
    {
        // LOGE("curl_easy_init error");
        std::cout << "curl_easy_init error" << std::endl;
        fclose(fp);
        return -2;
    }

    if (progress_data.resume)
    {
        curl_off_t from = progress_data.cur_file_size;
        curl_easy_setopt(curl, CURLOPT_RESUME_FROM_LARGE, from);
    }
    
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteFileCallback);
    curl_easy_setopt(curl, CURLOPT_NOPROGRESS, false);
    curl_easy_setopt(curl, CURLOPT_PROGRESSFUNCTION, ProgressCallback);
    progress_data.curl = curl;

    // progress_data.server = (MyServer*)user_data;
    curl_easy_setopt(curl, CURLOPT_PROGRESSDATA, &progress_data);
    if (NULL == ca_path)
    {
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0);
    }
    else
    {
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2);//The default value for this option is 2. 
        ///curl_easy_setopt(curl, CURLOPT_CAINFO, ca_path);
        curl_easy_setopt(curl, CURLOPT_CAPATH, ca_path);
    }
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1);
    
    CURLcode ret = curl_easy_perform(curl);
    if (ret != CURLE_OK)
    {
        // LOGE("curl_easy_perform error CURLcode = %d", ret);
        curl_easy_cleanup(curl);
        fclose(fp);
        return -3;
    }

    long code = 0;
    ret = curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
    if ((ret == CURLE_OK) && ((200 == code) || (response_code == code)))
    {
        curl_easy_cleanup(curl);
        fclose(fp);
        return 0;
    }
    else
    {
        // LOGE("curl_easy_getinfo error code = %ld, ret = %d", code, ret);

        std::cout << "curl_easy_getinfo error code = " << code << ", ret = " << ret << std::endl;

        curl_easy_cleanup(curl);
        fclose(fp);
        return -4;
    }

    return 0;
}

int Client::GetFileSize(const std::string& url, long long& file_size, const char* ca_path)
{
    file_size = 0;
    CURL *curl = curl_easy_init();
    if (NULL == curl)
    {
        std::cout << "curl_easy_init error" << std::endl;
        return -1;
    }

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HEADER, 1);
    curl_easy_setopt(curl, CURLOPT_NOBODY, 1);

    if (NULL == ca_path)
    {
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0);
    }
    else
    {
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1);
        curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 2);
        curl_easy_setopt(curl, CURLOPT_CAPATH, ca_path);
    }

    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1);

    CURLcode ret = curl_easy_perform(curl);
    if (ret != CURLE_OK)
    {
        // LOGE("curl_easy_perform error CURLcode = %d", ret);

        std::cout << "curl_easy_perform error CURL code = " << ret << std::endl;
        curl_easy_cleanup(curl);
        return -2;
    }

    curl_off_t cl;
    ret = curl_easy_getinfo(curl, CURLINFO_CONTENT_LENGTH_DOWNLOAD_T, &cl);
    if (ret == CURLE_OK)
    {
        curl_easy_cleanup(curl);
        file_size = cl;
        return 0;
    }
    else
    {
        // LOGE("curl_easy_getinfo ret = %d", ret);
         std::cout << "curl_easy_getinfo ret = " << ret << std::endl;
        curl_easy_cleanup(curl);
        return -3;
    }

    return 0;
}

}  // namespace cyberdog