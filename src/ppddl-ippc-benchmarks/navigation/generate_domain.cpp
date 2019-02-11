#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace std;


int main()
{


	ofstream domain_file;
	vector< vector<double> > Pr;


	
	string action_name;

	int X = 4;
	int Y = 3;

	int x0 = 3;
	int y0 = 0;

	int xf = 3;
	int yf = 2;

	vector<double> safe;
	for(int i=0;i<X; i++)
	{
		safe.push_back(1);
	}

	Pr.push_back(safe);


	vector<double> middle;
	middle.push_back(0.2);
	middle.push_back(0.4);
	middle.push_back(0.6);
	middle.push_back(0.8);


	for(int i=1; i<Y-1; i++)
	{
		cout<<"Here"<<endl;
		Pr.push_back(middle);
	}

	Pr.push_back(safe);

	cout<<Pr[0][1]<<endl;

	for(int y = 0; y < Y; y++)
	{
		for(int x = 0; x < X; x++)
		{
			cout<<x<<"-"<<y<<" ";
			cout<<Pr[y][x]<<" ";
			
		}
		cout<<endl;
	}


	string file_name = "domain_nav_"+to_string(X)+"_"+to_string(Y)+".pddl"; 
	domain_file.open(file_name);

	domain_file<<"(define (domain navigation)"<<endl;
  	domain_file<<"\t (:requirements :typing :strips :equality :probabilistic-effects)"<<endl;
  	domain_file<<"\t (:types posX posY)"<<endl;
  	domain_file<<"\t (:predicates"<<endl;
  	domain_file<<"\t \t (robot_at_X ?X - posX)"<<endl;
  	domain_file<<"\t \t (robot_at_Y ?Y - posY)"<<endl;
  	domain_file<<"\t \t (path-X-Y ?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)"<<endl;
  	
  	domain_file<<"\t \t (robot-dead)"<<endl;
  	domain_file<<"\t )"<<endl;
  	domain_file<<"\t (:functions (total-cost))"<<endl;


  	for(int y=0; y<Y; y++)
  	{
  		for(int x=0; x<X; x++)
  		{

  			//MOVE_UP

  			if(y!=Y-1)
  			{

  			action_name = "move_up_" +  to_string(x) + "_" + to_string(y);
  			domain_file<<"\t (:action "+ action_name<<endl;
  			domain_file<<"\t \t :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)"<<endl;
  			domain_file<<"\t \t :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))"<<endl;
  			domain_file<<"\t \t :effect (and (increase (total-cost) 1) (robot_at_Y ?toY) (not (robot_at_Y ?fromY)) (probabilistic "<<Pr[y+1][x]<<" (robot-dead)) )"<<endl;
  			domain_file<<"\t )"<<endl;

  			}

  			//MOVE_DOWN
  			if(y!=0)
  			{

  			action_name = "move_down_" +  to_string(x) + "_" + to_string(y);
  			domain_file<<"\t (:action "+ action_name<<endl;
  			domain_file<<"\t \t :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)"<<endl;
  			domain_file<<"\t \t :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))"<<endl;
  			domain_file<<"\t \t :effect (and (increase (total-cost) 1) (robot_at_Y ?toY) (not (robot_at_Y ?fromY)) (probabilistic "<<Pr[y-1][x]<<" (robot-dead)) )"<<endl;
  			domain_file<<"\t )"<<endl;

  			}

  			//MOVE_RIGHT

  			if(x!=X-1)
  			{
  			action_name = "move_right_" +  to_string(x) + "_" + to_string(y);
  			domain_file<<"\t (:action "+ action_name<<endl;
  			domain_file<<"\t \t :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)"<<endl;
  			domain_file<<"\t \t :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))"<<endl;
  			domain_file<<"\t \t :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic "<<Pr[y][x+1]<<" (robot-dead)) )"<<endl;
  			domain_file<<"\t )"<<endl;
  			}

  			//MOVE_LEFT
  			if(x!=0)
  			{
  			action_name = "move_left_" +  to_string(x) + "_" + to_string(y);
  			domain_file<<"\t (:action "+ action_name<<endl;
  			domain_file<<"\t \t :parameters (?fromX - posX ?fromY - posY ?toX - posX ?toY - posY)"<<endl;
  			domain_file<<"\t \t :precondition (and (not (robot-dead)) (robot_at_X ?fromX) (robot_at_Y ?fromY) (path-X-Y ?fromX ?fromY ?toX ?toY))"<<endl;
  			domain_file<<"\t \t :effect (and (increase (total-cost) 1) (robot_at_X ?toX) (not (robot_at_X ?fromX)) (probabilistic "<<Pr[y][x-1]<<" (robot-dead)) )"<<endl;
  			domain_file<<"\t )"<<endl;
  			}

  		}
  	}


  	domain_file<<")"<<endl;



  	file_name = "problem_nav_"+to_string(X)+"_"+to_string(Y)+".pddl"; 

  	ofstream problem_file;
  	problem_file.open(file_name);


  	problem_file<<"(define (problem navP)"<<endl;
  	problem_file<<"\t (:domain navigation)"<<endl;
  	problem_file<<"\t (:objects "<<endl;

  	problem_file<<"\t\t";
  	for(int x=0;x<X;x++)
  		problem_file<<"x-"<<x<<" ";

  	problem_file<<" - posX"<<endl;

  	problem_file<<"\t\t";
  	for(int y=0;y<Y;y++)
  		problem_file<<"y-"<<y<<" ";
  	problem_file<<" - posY \n \t )"<<endl;


  	problem_file<<"\t (:init (robot_at_X x-3) (robot_at_Y y-0) (not (robot-dead))"<<endl;

    for(int y=0; y<Y; y++)
  	{
  		for(int x=0; x<X; x++)
  		{

  			if(x!=X-1)
  				problem_file<<"\t\t (path-X-Y x-"<<to_string(x)<<" "<<"y-"<<to_string(y)<<" x-"<<to_string(x+1)<<" y-"<<to_string(y)<<")"<<endl;

  			if(x!=0)
  				problem_file<<"\t\t (path-X-Y x-"<<to_string(x)<<" "<<"y-"<<to_string(y)<<" x-"<<to_string(x-1)<<" y-"<<to_string(y)<<")"<<endl;

  			if(y!=Y-1)
  				problem_file<<"\t\t (path-X-Y x-"<<to_string(x)<<" "<<"y-"<<to_string(y)<<" x-"<<to_string(x)<<" y-"<<to_string(y+1)<<")"<<endl;

  			if(y!=0)
  				problem_file<<"\t\t (path-X-Y x-"<<to_string(x)<<" "<<"y-"<<to_string(y)<<" x-"<<to_string(x)<<" y-"<<to_string(y-1)<<")"<<endl;
  		}
  	}

	problem_file<<"\t )"<<endl;
  	problem_file<<"\t (:goal (and (robot-at x-3 y-2) (not (robot-dead))))"<<endl;
  	problem_file<<"\t (:metric minimize (total-cost))"<<endl;
	problem_file<<")"<<endl;




	return 0;
}
