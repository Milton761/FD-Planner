#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>

using namespace std;


int main(int argc, char** argv)
{


	bool flag = false;


	string file_name = argv[1];


    ifstream iFile(file_name);    // input.txt has integers, one per line


    int instance = 1;
    string maxprob;
    string updates;
    string time;
    string memory;
    string states;

    int features = 0;

	string keyword;
	string value;
	string spam;


    cout<<setw(15)<<"name"<<setw(15)<<"states"<<setw(15)<<"utility"<<setw(15)<<"time"<<setw(15)<<"memory"<<setw(15)<<"updates"<<endl;
    while (!iFile.eof())
    {
    	if(features==5)
    	{
    		cout<<setw(15)<<instance<<setw(15)<<states<<setw(15)<<maxprob<<setw(15)<<time<<setw(15)<<memory<<setw(15)<<updates<<endl;
    		features=0;
    		instance++;
    	}

 
    	iFile >> keyword;

    	
    	if(keyword == "Prob")
    	{
    		iFile>>keyword;
    		iFile>>keyword;
    		iFile>>keyword;
    		iFile>>states;
    		
    		features++;
    	}

    	if(keyword == "updates:")
    	{
    		iFile>>updates;
    		features++;
    		
    	}

    	if(keyword == "Search")
    	{
    		iFile>>keyword;
    		if(keyword == "time:")
    		{
    			iFile>>time;

    			time.pop_back();
    			features++;
    			
    		}
    	}


    	

    	if(keyword == "Final")
    	{
    		iFile>>keyword;
    		if(keyword == "value")
    		{
    			iFile>>spam;
    			iFile>>maxprob;
    			maxprob.erase(0,1);
    			features++;
    		}
    	}

    	if(keyword == "Peak")
    	{
    		
    		iFile>>keyword;
    		if(keyword == "memory:")
    		{
    			//memory.pop_back();

    			if(features==4)
    			{
    				iFile>>memory;
    				features++;
    				
    			}else{
    				features=0;
    			}
    			
    		}
    	}

    	

    }


	
    return 0;


}
