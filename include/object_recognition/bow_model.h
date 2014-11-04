#ifndef BOW_MODEL_H
#define BOW_MODEL_H

#include <vector>
#include <string>

struct Object_Model{};

/**
 * @brief Class that contains the model for the Bag of Words algorithm.
 * It provides operations to load from and save to .txt file, as well
 * as update operations for the model
 */
class BoW_Model
{
public:
    /**
     * @brief Default constructor
     */
    BoW_Model();

    /**
     * @brief Copy constructor
     * @param src
     */
    BoW_Model(const std::vector<Object_Model> &src);

    /**
     * @brief Loads a model from a text file
     * @param path path to the text file describing the model
     */
    void load_model(const std::string &path);

    /**
     * @brief Saves the model to a file
     * @param path path to the file where the model is saved
     */
    void save_model(const std::string &path) const;

    /**
     * @brief Inserts a new object into the model
     * @param src the new object
     */
    void add_object(const Object_Model &src);

    /**
     * @brief Getter for the model
     * @return the model as a vector of objects
     */
    const std::vector<Object_Model>& get_Model() const;

private:
    std::vector<Object_Model> model_;
};

#endif // BOW_MODEL_H
