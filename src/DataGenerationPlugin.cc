#include <cmath>
#include <gazebo/gazebo.hh>
#include "gazebo/physics/physics.hh"
#include <ignition/math/Pose3.hh>
#include "gazebo/common/common.hh"
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <string>

using namespace std;

namespace gazebo
{
  class DataGenerationPlugin : public WorldPlugin
  {
    const int ROCK_COUNT = 7;
    const int UXO_COUNT = 4;

    const double SONAR_Z = 2;
    const double SONAR_RANGE = 10;
    const int IMG_WIDTH = 512;
    const int IMG_HEIGHT = 399;

    int iteration = 0;
    common::Time lastIterTime;
    bool firstIter = true;

    private:
        physics::WorldPtr world;
        default_random_engine random;
        uniform_real_distribution<double> randomAngle;

        vector<physics::ModelPtr> rockModels;
        vector<physics::ModelPtr> uxoModels;


    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        this->world = _world;

        this->randomAngle = uniform_real_distribution<double>(0, 2 * IGN_PI);

        // Initialize ROS
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "dataset_generation_plugin");
        }

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DataGenerationPlugin::OnUpdate, this, _1));

        for (int i = 1; i <= ROCK_COUNT; i++) {
            rockModels.push_back(this->world->ModelByName("rock" + to_string(i)));
        }

        for (int i = 1; i <= UXO_COUNT; i++) {
            uxoModels.push_back(this->world->ModelByName("uxo" + to_string(i)));
        }

        ROS_INFO("Dataset generation plugin loaded!");

    }

    private: pair<double, double> ProjectPointToSonarImage(double x, double y, double z) {
        double scaleFactor = sqrt((x * x + y * y + pow(SONAR_Z - z, 2)) / (x * x + y * y));

        double camX = -y * scaleFactor;
        double camY = x * scaleFactor;

        double pixPerM = IMG_HEIGHT / SONAR_RANGE;

        double pixelX = camX * pixPerM + IMG_WIDTH / 2;
        double pixelY = -camY * pixPerM + IMG_HEIGHT;

        return pair<double, double>(pixelX, pixelY);
    }

    public: void OnUpdate(const common::UpdateInfo &_info)
    {
        if (iteration > 10) {
            world->SetPaused(true);
            return;
        }

        // Wait until iterations start taking longer, indicating the sonar is running, to increment the iteration count
        common::Time currentTime = this->world->RealTime();
        if (firstIter || currentTime.Double() - lastIterTime.Double() < 0.1) {
            firstIter = false;
        } else {
            iteration++;
        }

        ROS_INFO_STREAM("Running iteration " + to_string(iteration));

        physics::ModelPtr sonarModel = this->world->ModelByName("sonar");
        physics::ModelPtr groundModel = this->world->ModelByName("ocean_floor");

        std::uniform_real_distribution<double> dist(-10, 10);
        ignition::math::Pose3d floorPose(dist(this->random), dist(this->random), 0, 0, 0, this->randomAngle(this->random));
        groundModel->SetWorldPose(floorPose);

        ignition::math::Pose3d nullPose(-10, 0, 5, 0, 0, 0);


        for (physics::ModelPtr rockModel : this->rockModels) {
            rockModel->SetWorldPose(nullPose);
        }

        for (physics::ModelPtr uxoModel : this->uxoModels) {
            uxoModel->SetWorldPose(nullPose);
        }

        int objectCount = rand() % 4 + 1;
        int rockCount = 0;
        int uxoCount = 0;
        for (int i = 0; i < objectCount; i++) {
            if (rand() % 3 == 0) uxoCount++;
            else rockCount++;
        }

        uxoCount = 1;
        rockCount = 0;

        vector<physics::ModelPtr> selectedObjects;
        vector<string> objectClasses;

        for (int i = 0; i < rockCount; i++) {
            objectClasses.push_back("rock");
            physics::ModelPtr rock;
            do {
                rock = this->rockModels[rand() % ROCK_COUNT];
            } while (find(selectedObjects.begin(), selectedObjects.end(), rock) != selectedObjects.end());
            selectedObjects.push_back(rock);
        }

        for (int i = 0; i < uxoCount; i++) {
            objectClasses.push_back("uxo");
            physics::ModelPtr uxo;
            do {
                uxo = this->uxoModels[rand() % UXO_COUNT];
                uxo = this->uxoModels[0];
            } while (find(selectedObjects.begin(), selectedObjects.end(), uxo) != selectedObjects.end());
            selectedObjects.push_back(uxo);
        }

        // Start writing xml file
        ofstream outfile;
        outfile.open("/home/noah/uuv_ws/src/dataset_generation/generated_dataset/annotations/" + to_string(iteration) + ".xml");
        outfile << "<annotation>" << endl;
        outfile << "    <folder>images</folder>" << endl;
        outfile << "    <filename>" + to_string(iteration) + ".png</filename>" << endl;
        outfile << "    <path>images/" + to_string(iteration) + ".png</path>" << endl;
        outfile << "    <size>" << endl;
        outfile << "        <width>" + to_string(IMG_WIDTH) + "</width>" << endl;
        outfile << "        <height>" + to_string(IMG_HEIGHT) + "</height>" << endl;
        outfile << "    </size>" << endl;


        int i = 0;
        for (physics::ModelPtr model : selectedObjects) {
            std::uniform_real_distribution<double> dist(0, 1);

            double r = dist(this->random) * 4 + 4;
            double theta = dist(this->random) * IGN_PI / 4 - IGN_PI / 8;
            double angle = this->randomAngle(this->random);

            double x = r * cos(theta);
            double y = r * sin(theta);

            const double MODEL_Z = 0.25;

            ignition::math::Pose3d objectPose(x, y, MODEL_Z, 0, 0, angle);

            model->SetWorldPose(objectPose);

            // Write annotation file
            auto boundingBox = model->BoundingBox();
            ROS_INFO_STREAM(boundingBox);

            pair<double, double> topLeftPoint = ProjectPointToSonarImage(boundingBox.Max().X(), boundingBox.Max().Y(), MODEL_Z);
            pair<double, double> botRightPoint = ProjectPointToSonarImage(boundingBox.Min().X(), boundingBox.Min().Y(), MODEL_Z);
            double imgX = topLeftPoint.first;
            double imgY = topLeftPoint.second;
            double imgWidth = botRightPoint.first - imgX;
            double imgHeight = botRightPoint.second - imgY;

            // ROS_INFO_STREAM(topLeftPoint.first << ", " << topLeftPoint.second);
            // ROS_INFO_STREAM(botRightPoint.first << ", " << botRightPoint.second);
            // ROS_INFO_STREAM(imgWidth << "x" << imgHeight);

            // Write object to annotation
            outfile << "    <object>" << endl;
            outfile << "        <name>" + objectClasses[i] + "</name>" << endl;
            outfile << "        <bndbox>" << endl;
            outfile << "            <x>" + to_string(imgX) + "</x>" << endl;
            outfile << "            <y>" + to_string(imgY) + "</y>" << endl;
            outfile << "            <w>" + to_string(imgWidth) + "</w>" << endl;
            outfile << "            <h>" + to_string(imgHeight) + "</h>" << endl;
            outfile << "        </bndbox>" << endl;
            outfile << "    </object>" << endl;

            i++;
        }

        // Close xml file
        outfile << "</annotation>" << endl;
        outfile.close();
    }

    private:
        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;

  };

  GZ_REGISTER_WORLD_PLUGIN(DataGenerationPlugin)
}
