#include <iostream>

#include "ogrsf_frmts.h"
#include "gdal.h"
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <fstream>
#include <utility>

/// Проверяет, существует ли файл
static bool fileExist(const std::string &name) {
    std::ifstream f(name.c_str());
    return f.good();
}

/// Позволяет работать как с относительными, так и с абсолютными путями
static std::string path(std::string pathStr) {
    if (fileExist(pathStr))
        return pathStr;
    if (fileExist("input/" + pathStr))
        return "input/" + pathStr;
    if (fileExist("output/" + pathStr))
        return "output/" + pathStr;
    throw std::invalid_argument("Такого файла не существуеть!");
}

/// Вывод выходных данных в файл
static void outputToFile(const std::vector<int> &output, std::string pathStr) {
    std::ofstream fileOutput;
    fileOutput.open(path(std::move(pathStr)));
    if (fileOutput.is_open()) {
        for (int i : output) {
            fileOutput << i << std::endl;
        }
    }
    fileOutput.close();
}

/// Вспомогательный метод к методу split
static bool space(char c) {
    return std::isspace(c);
}

/// Вспомогательный метод к методу split
static bool notSpace(char c) {
    return !std::isspace(c);
}

/// Метод, иммитирующий split в c#. Разделяет строку на слова и помещает их в вектор.
static std::vector<std::string> split(const std::string &s) {
    typedef std::string::const_iterator iter;
    std::vector<std::string> ret;
    iter i = s.begin();

    while (i != s.end()) {
        i = std::find_if(i, s.end(), notSpace);
        iter j = std::find_if(i, s.end(), space);

        if (i != s.end()) {
            ret.emplace_back(i, j);
            i = j;
        }
    }
    return ret;
}

/// Метод, считывает строку из файла
static std::string getInputString(std::string pathStr) {
    std::ifstream fileInput(path(std::move(pathStr)));
    std::string inputString;
    if (fileInput.is_open()) {
        if (getline(fileInput, inputString)) {}
    }
    fileInput.close();
    return inputString;
}

/// Метод загружает векторные данные, находит OSM_ID и MBR и заполняет дерево
static void readDataset(boost::geometry::index::rtree<std::pair<boost::geometry::model::box
        <boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>>, int>,
        boost::geometry::index::quadratic<8, 4>> &rt, const char *pathStr) {
    // с помощью GDAL загружаем векторные
    // данные из файла building-polygon.shp
    GDALAllRegister();
    auto *dataset = static_cast<GDALDataset *>(GDALOpenEx(
            pathStr,
            GDAL_OF_VECTOR,
            nullptr, nullptr, nullptr));

    if (dataset == nullptr) {
        std::cout << "Cannot open file!" << std::endl;
        return;
    }
    // считываем нулевой слой датасета
    auto &&layer = dataset->GetLayers()[0];
    int OSM_ID;
    for (auto &&feature : layer) {
        // Получаем OSM_ID
        OSM_ID = feature->operator[](0).GetAsInteger();
        // Ищем MBR
        const std::shared_ptr<OGREnvelope> polygonEnvelope(new OGREnvelope);
        auto *geometry = feature->GetGeometryRef();
        geometry->getEnvelope(polygonEnvelope.get());

        boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> minPoint(polygonEnvelope->MinX,
                                                                                          polygonEnvelope->MinY);
        boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> maxPoint(polygonEnvelope->MaxX,
                                                                                          polygonEnvelope->MaxY);
        boost::geometry::model::box<boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>> rectangle(
                minPoint, maxPoint);
        // Вставляем в дерево
        rt.insert(std::make_pair(rectangle, OSM_ID));
    }
}

/// Метод обрабатывает входные координаты, находит пересекаемые прямоугольником MBR геометрических объектов
/// и возвращает их атрибуты OSM_ID
static std::vector<int> findIntersection(boost::geometry::index::rtree<std::pair<boost::geometry::model::box
        <boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>>, int>,
        boost::geometry::index::quadratic<8, 4>> &rt, std::vector<std::string> input) {
    // обрабатываем входные координаты
    boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> minPoint(std::stod(input[0]),
                                                                                      std::stod(input[1]));
    boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> maxPoint(std::stod(input[2]),
                                                                                      std::stod(input[3]));
    boost::geometry::model::box<boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>> rectangle(
            minPoint, maxPoint);

    std::vector<std::pair<boost::geometry::model::box<boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>>, int>> output;

    // поиск пересечений
    rt.query(boost::geometry::index::intersects(rectangle), back_inserter(output));

    // атрибуты OSM_ID пересекаемых прямоугольником MBR геометрических объектов
    std::vector<int> outputInt;
    for (auto &i : output) {
        outputInt.push_back(i.second);
    }
    return outputInt;
}

int main(int argc, char *argv[]) {
    // создаем R-дерево
    boost::geometry::index::rtree<std::pair<boost::geometry::model::box
            <boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>>, int>,
            boost::geometry::index::quadratic<8, 4>> rt;
    // загружаем векторные данные, находим OSM_ID и MBR и заполняем дерево
    readDataset(rt, argv[1]);
    std::vector<std::string> input = split(getInputString(argv[2]));
    // обрабатываем входные координаты, находим пересекаемые прямоугольником MBR
    // геометрических объектов и возвращаем их атрибуты OSM_ID
    std::vector<int> outputInt = findIntersection(rt, input);
    std::sort(outputInt.begin(), outputInt.end());
    outputToFile(outputInt, argv[3]);
    return 0;
}